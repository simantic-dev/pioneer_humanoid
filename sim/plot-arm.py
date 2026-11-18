# Plots the MuJoCo-arm run from sim/out/arm-sim.log (ARM t=... lines every 10 ms of virtual time)
# and the CAN master's two commands (times from sim/peers/can_peer_arm.py). Output: sim/screenshots/arm-step.png
import math, re, sys, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "peers"))
from can_peer_arm import STEPS
rows = [tuple(map(float, m.groups())) for m in
        (re.search(r"ARM t=(\S+) q=(\S+) qd=(\S+) tau=(\S+) iq=(\S+)", l) for l in open("sim/out/arm-sim.log", errors="ignore")) if m]
t, q, qd, tau, iq = zip(*rows)
tgt = [0.0 if x < STEPS[0][0] / 1e6 else math.pi * int(STEPS[0][1]) / 100 if x < STEPS[1][0] / 1e6 else math.pi * int(STEPS[1][1]) / 100 for x in t]
fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax[0].plot(t, tgt, "k--", lw=1, label="CAN target"); ax[0].plot(t, q, label="hinge angle (MuJoCo)")
ax[0].set_ylabel("rad"); ax[0].legend(loc="upper left"); ax[0].set_title("Pioneer joint firmware closing the loop on a MuJoCo arm (virtual time)")
ax[1].plot(t, tau, color="C3"); ax[1].set_ylabel("motor torque, N*m"); ax[1].set_xlabel("virtual time, s")
for a in ax:
    for s, _ in STEPS: a.axvline(s / 1e6, color="gray", lw=0.5)
fig.tight_layout(); os.makedirs("sim/screenshots", exist_ok=True); fig.savefig("sim/screenshots/arm-step.png", dpi=110)
print(len(rows), "samples")
ok = True
for (s0, nn), (s1, _) in zip(STEPS, STEPS[1:] + [(STEPS[-1][0] + 2_300_000, b"")]):
    tg = math.pi * int(nn) / 100
    win = [x - tg for tt, x in zip(t, q) if s1 / 1e6 - 0.5 <= tt <= s1 / 1e6]
    pk = max(q[i] for i in range(len(t)) if s0 / 1e6 <= t[i] <= s1 / 1e6)
    start = [q[i] for i in range(len(t)) if t[i] <= s0 / 1e6][-1]
    print(f"step to {tg:.3f} rad: peak {pk:.3f} (overshoot {(pk - tg) / (tg - start) * 100:+.1f}% of the step), last 0.5 s: mean error {sum(win)/len(win):+.4f}, max |error| {max(abs(w) for w in win):.4f} rad")
    ok &= max(abs(w) for w in win) < 0.05
print("SETTLED" if ok else "NOT SETTLED (max |error| over the last 0.5 s of a hold >= 0.05 rad)")
