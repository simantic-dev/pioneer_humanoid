# One GIF of every case from run_cases.py, played together on one virtual clock: each panel is MuJoCo's own render of that case's arm (set to the
# logged joint angles; the green ghost is the commanded angle), with the true load-angle trace underneath. Nothing is re-simulated.
# Needs sim/out/cases/*/ from run_cases.py. Output: sim/screenshots/arm-cases.gif and .mp4
import json, math, os, re, subprocess, sys
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "sim", "peers"))
from cases import CASES
from can_peer_arm import STEPS
R = json.load(open(os.path.join(ROOT, "sim/out/cases/results.json")))
tg = lambda x: 0.0 if x < STEPS[0][0] / 1e6 else math.pi * int(STEPS[0][1]) / 100 if x < STEPS[1][0] / 1e6 else math.pi * int(STEPS[1][1]) / 100
STRIDE, W, H = 8, 260, 190
data = {}
for c in CASES:
    d = os.path.join(ROOT, "sim/out/cases", c["name"])
    rows = [tuple(map(float, m.groups())) for m in (re.search(r"ARM t=(\S+) q=(\S+) qd=\S+ tau=\S+ iq=\S+ qr=(\S+)", l) for l in open(os.path.join(d, "arm-sim.log"), errors="ignore")) if m]
    t, q, qr = map(np.array, zip(*rows))
    m = mujoco.MjModel.from_xml_path(os.path.join(d, "arm.xml")); dd = mujoco.MjData(m); r = mujoco.Renderer(m, H, W)
    two = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rotor_j") >= 0
    il, ir = (int(m.joint("load_j").qposadr[0]), int(m.joint("rotor_j").qposadr[0])) if two else (0, 0)
    ghost = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ghost"); imgs = []
    for i in range(0, len(t), STRIDE):
        dd.qpos[il] = q[i]; dd.qpos[ir] = qr[i] if two else q[i]
        a = tg(t[i]); m.body_quat[ghost] = [math.cos(a / 2), 0, math.sin(a / 2), 0]
        mujoco.mj_forward(m, dd); r.update_scene(dd, camera="side"); imgs.append(r.render().copy())
    r.close(); data[c["name"]] = dict(t=t, q=q, imgs=imgs)
n = len(CASES); cols = 4; rows_n = math.ceil(n / cols)
fig = plt.figure(figsize=(15.6, 4.7 * rows_n + 0.5), dpi=80)
pw, ph = 1 / cols, (1 - 0.06) / rows_n
art = []
for k, c in enumerate(CASES):
    r_, c_ = divmod(k, cols); x0, y0 = c_ * pw, 1 - 0.06 - (r_ + 1) * ph
    ax = fig.add_axes([x0 + 0.006, y0 + ph * 0.40, pw - 0.012, ph * 0.40]); ax.axis("off")
    im = ax.imshow(data[c["name"]]["imgs"][0])
    fig.text(x0 + 0.01, y0 + ph - 0.01, c["label"], fontsize=10, weight="bold", va="top")
    fig.text(x0 + 0.01, y0 + ph - 0.045, "\n".join(__import__("textwrap").wrap(c["params"], 44)), fontsize=7.2, va="top", color="#333")
    tr = fig.add_axes([x0 + 0.03, y0 + ph * 0.115, pw - 0.045, ph * 0.255]); tr.set_xlim(0, 11); tr.set_ylim(min(-0.2, float(data[c['name']]['q'].min()) - 0.2), 2.2); tr.tick_params(labelsize=6)
    tr.plot(data[c["name"]]["t"], [tg(x) for x in data[c["name"]]["t"]], "k--", lw=0.8)
    ln, = tr.plot([], [], color="C0", lw=1.1)
    res = R[c["name"]]
    if res.get("ready_s") is None: cap = "firmware did not reach 'Motor ready'"
    else:
        s2 = res["step2"]; st = "none" if s2["settle_2cm_s"] is None else f"{s2['settle_2cm_s']:.2f} s"
        cap = f"step 2: overshoot {s2['overshoot_pct']:+.1f}%  settle {st}\nlast 0.5 s: error {s2['mean_err_last0p5']:+.3f}, p-p {s2['pp_last0p5']:.3f} rad"
    fig.text(x0 + 0.01, y0 + 0.004, cap, fontsize=7, va="bottom", family="monospace")
    art.append((im, ln, c["name"]))
head = fig.text(0.01, 0.985, "", fontsize=11, weight="bold", va="top")
fig.text(0.99, 0.985, "same firmware binary in every panel; MuJoCo arm and sensor change; playback of captured runs, virtual time", fontsize=8, ha="right", va="top", color="gray")
frames = []
nf = min(len(v["imgs"]) for v in data.values())
for i in range(nf):
    for im, ln, nm in art:
        im.set_data(data[nm]["imgs"][i]); j = min(i * STRIDE, len(data[nm]["t"]) - 1)
        ln.set_data(data[nm]["t"][:j + 1], data[nm]["q"][:j + 1])
    head.set_text(f"Pioneer joint firmware on pyrite + MuJoCo: t = {data[CASES[0]['name']]['t'][min(i * STRIDE, len(data[CASES[0]['name']]['t']) - 1)]:5.2f} s")
    fig.canvas.draw(); frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
h, w, _ = frames[0].shape
out = os.path.join(ROOT, "sim/screenshots"); os.makedirs(out, exist_ok=True)
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "12.5", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "27", os.path.join(out, "arm-cases.mp4")], stdin=subprocess.PIPE)
for f in frames: p.stdin.write(f.tobytes())
p.stdin.close(); p.wait()
g = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=48) for f in frames]
g[0].save(os.path.join(out, "arm-cases.gif"), save_all=True, append_images=g[1:], duration=80, loop=0, optimize=True)
print(len(frames), "frames", w, "x", h)
