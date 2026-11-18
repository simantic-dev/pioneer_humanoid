# Renders a captured run through MuJoCo's own renderer, next to what the simulated MCU did, from
# sim/out/arm-sim.log (ARM t= samples: hinge angle, torque, current) and sim/out/arm-uart.log (the
# firmware's USB console, virtual-time stamped). Nothing is re-simulated: each frame sets the hinge to
# the logged angle and calls mj_forward. The green ghost is the commanded angle. One frame per 40 ms of virtual time.
import math, os, re, sys, subprocess
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "peers"))
from can_peer_arm import STEPS
rows = [tuple(map(float, m.groups())) for m in
        (re.search(r"ARM t=(\S+) q=(\S+) qd=\S+ tau=(\S+) iq=(\S+)", l) for l in open("sim/out/arm-sim.log", errors="ignore")) if m]
t, q, tau, iq = map(np.array, zip(*rows))
tg = lambda x: 0.0 if x < STEPS[0][0]/1e6 else math.pi*int(STEPS[0][1])/100 if x < STEPS[1][0]/1e6 else math.pi*int(STEPS[1][1])/100
con = []
for l in open("sim/out/arm-uart.log", errors="ignore"):
    m = re.match(r"\[([\d.]+)s\] \(usb\) (.*)", l.strip())
    if m and re.search(r"MOT:(Align|sensor dir|Zero|Enable|Ready)|Motor ready|STM32 Serial", m.group(2)): con.append((float(m.group(1)), m.group(2)))
for s, nn in STEPS: con.append((s/1e6, f"CAN 0x123 <- '{nn.decode()}' = {math.pi*int(nn)/100:.3f} rad"))
con.sort()
m = mujoco.MjModel.from_xml_path("sim/mujoco/arm.xml"); d = mujoco.MjData(m)
ghost = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ghost")
r = mujoco.Renderer(m, 420, 560)
frames = []
for i in range(0, len(t), 4):
    d.qpos[0] = q[i]
    a = tg(t[i]); m.body_quat[ghost] = [math.cos(a/2), 0, math.sin(a/2), 0]
    mujoco.mj_forward(m, d); r.update_scene(d, camera="side"); img = r.render()
    fig = plt.figure(figsize=(10, 5.6), dpi=80)
    ax = fig.add_axes([0.02, 0.36, 0.5, 0.58]); ax.imshow(img); ax.axis("off")
    fig.text(0.03, 0.95, "MuJoCo 4.0.1: the arm (gravity, inertia, joint)", fontsize=10, weight="bold")
    b = fig.add_axes([0.6, 0.45, 0.37, 0.43])
    b.plot(t[:i+1], q[:i+1], color="C0"); b.plot(t[:i+1], [tg(x) for x in t[:i+1]], "k--", lw=1)
    b.set_xlim(0, t[-1]); b.set_ylim(-0.2, 2.1); b.set_xlabel("virtual time, s", fontsize=8); b.set_ylabel("hinge angle, rad", fontsize=8); b.tick_params(labelsize=8)
    fig.text(0.6, 0.95, f"t = {t[i]:5.2f} s   torque {tau[i]:+.2f} N*m   iq {iq[i]:+.1f} A", fontsize=9, family="monospace")
    fig.text(0.03, 0.30, "pyrite 0.10.4: the firmware on a simulated STM32G474 (its USB console, CAN, SPI encoder, PWM)", fontsize=10, weight="bold")
    lines = [f"[{ts:6.3f}s] {tx}" for ts, tx in con if ts <= t[i]][-6:]
    fig.text(0.03, 0.04, "\n".join(lines), fontsize=9, family="monospace", va="bottom")
    fig.text(0.99, 0.01, "playback of a captured run, virtual time", fontsize=7, ha="right", color="gray")
    fig.canvas.draw(); frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()); plt.close(fig)
h, w, _ = frames[0].shape
os.makedirs("sim/screenshots", exist_ok=True)
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "25", "-i", "-",
                      "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", "sim/screenshots/arm-mujoco.mp4"], stdin=subprocess.PIPE)
for f in frames: p.stdin.write(f.tobytes())
p.stdin.close(); p.wait()
g = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
g[0].save("sim/screenshots/arm-mujoco.gif", save_all=True, append_images=g[1:], duration=40, loop=0, optimize=True)
print(len(frames), "frames", w, "x", h)
