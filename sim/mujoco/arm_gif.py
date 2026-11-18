# Playback GIF built only from sim/out/arm-sim.log (the ARM t=... samples of a captured run): the arm
# drawn at the logged hinge angle, the angle trace revealed as virtual time advances.
import math, re, sys, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "peers"))
from can_peer_arm import STEPS
rows = [tuple(map(float, m.groups())) for m in
        (re.search(r"ARM t=(\S+) q=(\S+) qd=\S+ tau=(\S+)", l) for l in open("sim/out/arm-sim.log", errors="ignore")) if m]
t, q, tau = zip(*rows)
tg = lambda x: 0.0 if x < STEPS[0][0]/1e6 else math.pi*int(STEPS[0][1])/100 if x < STEPS[1][0]/1e6 else math.pi*int(STEPS[1][1])/100
frames = []
for i in range(0, len(t), 10):
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 3.2), gridspec_kw={"width_ratios": [1, 1.6]})
    a.set_xlim(-0.4, 0.4); a.set_ylim(-0.4, 0.15); a.set_aspect("equal"); a.axis("off")
    a.plot([0, 0.35 * math.sin(tg(t[i]))], [0, -0.35 * math.cos(tg(t[i]))], ":", color="gray")
    tip = (0.3 * math.sin(q[i]), -0.3 * math.cos(q[i]))
    a.plot([0, tip[0]], [0, tip[1]], "-", lw=3, color="C0"); a.plot(*tip, "o", ms=12, color="C0"); a.plot(0, 0, "ko")
    a.set_title(f"t = {t[i]:.2f} s   torque {tau[i]:+.2f} N*m", fontsize=9)
    b.plot(t[:i+1], q[:i+1], color="C0"); b.plot(t[:i+1], [tg(x) for x in t[:i+1]], "k--", lw=1)
    b.set_xlim(0, t[-1]); b.set_ylim(-0.2, 2.1); b.set_xlabel("virtual time, s"); b.set_ylabel("hinge angle, rad")
    fig.suptitle("Playback of a captured run: Pioneer joint firmware on MuJoCo (virtual time)", fontsize=8)
    fig.tight_layout(); fig.canvas.draw()
    frames.append(Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert("P", palette=Image.ADAPTIVE, colors=64)); plt.close(fig)
frames[0].save("sim/screenshots/arm-playback.gif", save_all=True, append_images=frames[1:], duration=90, loop=0, optimize=True)
print(len(frames), "frames")
