# Runs every case in cases.py through the firmware in sim, one after another, and scores each from MuJoCo's true load angle.
# Usage: VENV=<venv> python sim/mujoco/run_cases.py [case ...]   -> sim/out/cases/<name>/{arm.xml,arm-sim.log,arm-uart.log}, sim/out/cases/results.json
import json, math, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "sim", "peers"))
import arm_xml
from cases import CASES
from can_peer_arm import STEPS

def score(t, q, er):
    out = {}
    ends = [s / 1e6 for s, _ in STEPS[1:]] + [t[-1]]
    q0 = 0.0
    for k, ((s, nn), e) in enumerate(zip(STEPS, ends), 1):
        tg = math.pi * int(nn) / 100; s = s / 1e6
        i0 = max(i for i in range(len(t)) if t[i] <= s); start = q[i0]; delta = tg - start
        idx = [i for i in range(len(t)) if s <= t[i] <= e]
        seg_t = [t[i] - s for i in idx]; seg = [q[i] for i in idx]
        def cross(frac):
            for tt, x in zip(seg_t, seg):
                if (x - start) / delta >= frac: return tt
        t10, t90 = cross(0.1), cross(0.9)
        peak = max(seg); band = 0.02  # rad
        out_of = [tt for tt, x in zip(seg_t, seg) if abs(x - tg) > band]
        settle = None if (out_of and out_of[-1] >= seg_t[-1] - 0.02) else (out_of[-1] if out_of else 0.0)
        last = [x - tg for tt, x in zip(seg_t, seg) if tt >= seg_t[-1] - 0.5]
        lat = next((tt for tt, x in zip(seg_t, seg) if abs(x - start) > 0.02 * abs(delta)), None)
        out[f"step{k}"] = dict(target=tg, rise_10_90_s=None if t10 is None or t90 is None else t90 - t10, overshoot_pct=(peak - tg) / delta * 100,
                               settle_2cm_s=settle, mean_err_last0p5=sum(last) / len(last), pp_last0p5=max(last) - min(last), latency_s=lat)
    # firmware loop rate: encoder angle reads per second, over 5.2 s to 5.9 s (after init, before the first command)
    a, b = [i for i in range(len(t)) if 5.2 <= t[i] <= 5.9][0], [i for i in range(len(t)) if 5.2 <= t[i] <= 5.9][-1]
    out["loop_rate_hz"] = (er[b] - er[a]) / (t[b] - t[a])
    return out

def main():
    venv = os.environ["VENV"]; sp = [os.path.join(venv, "lib", d, "site-packages") for d in os.listdir(os.path.join(venv, "lib"))][0]
    want = sys.argv[1:] or [c["name"] for c in CASES]
    res = {}
    os.makedirs(os.path.join(ROOT, "sim/out/cases"), exist_ok=True)
    for c in CASES:
        if c["name"] not in want: continue
        d = os.path.join(ROOT, "sim/out/cases", c["name"]); os.makedirs(d, exist_ok=True)
        open(os.path.join(d, "arm.xml"), "w").write(arm_xml.make(**c["arm"]))
        env = dict(os.environ, PYTHONPATH=sp, ARM_XML=os.path.join(d, "arm.xml"), ENC_OFFSET_RAD="3.14159")
        env.update({k: str(v) for k, v in c["env"].items()})
        t0 = time.time()
        cmd = ["lockf", "-k", "/Users/anoof/dev/simantic/.heavy.lock", "timeout", "300", "sim", "run", "--scenario", c.get("scenario", "sim/scenario-arm.yaml"), "--usb-cdc-console", "dut.usb",
               "--timeout", "11", "--ascii", "--expect", "Motor ready!!!"]
        if os.environ.get("RESCORE") and os.path.exists(os.path.join(d, "arm-sim.log")):  # score the logs already there
            r = type("R", (), {"returncode": 0})
        else:
            r = subprocess.run(cmd, cwd=ROOT, env=env, stdout=open(os.path.join(d, "arm-uart.log"), "w"), stderr=open(os.path.join(d, "arm-sim.log"), "w"))
        rows = [tuple(map(float, m.groups())) for m in (re.search(r"ARM t=(\S+) q=(\S+) qd=\S+ tau=(\S+) iq=\S+ qr=(\S+) er=(\S+)", l)
                for l in open(os.path.join(d, "arm-sim.log"), errors="ignore")) if m]
        uart = open(os.path.join(d, "arm-uart.log"), errors="ignore").read()
        ready = re.search(r"\[([\d.]+)s\] \(usb\) Motor ready!!!", uart)
        notes = sorted(set(re.findall(r"\(usb\) ((?:WARN|ERR)-MOT:[^\n]*)", uart)))
        res[c["name"]] = dict(exit=r.returncode, wall_s=round(time.time() - t0, 1), ready_s=float(ready.group(1)) if ready else None, firmware_notes=notes)
        if rows and ready:
            t, q, tau, qr, er = zip(*rows); res[c["name"]].update(score(t, q, er)); res[c["name"]]["peak_torque"] = max(map(abs, tau))
        print(c["name"], json.dumps(res[c["name"]])[:300], flush=True)
        path = os.path.join(ROOT, "sim/out/cases/results.json"); old = json.load(open(path)) if os.path.exists(path) else {}; old.update(res); json.dump(old, open(path, "w"), indent=1)
    path = os.path.join(ROOT, "sim/out/cases/results.json")
    old = json.load(open(path)) if os.path.exists(path) else {}
    old.update(res); json.dump(old, open(path, "w"), indent=1)
main()
