# Sim addition. Renders sim/screenshots/*.png from sim/out/{uart.log,sim.log,can-frames.txt}, the run's own
# captures (sim/run.sh). Needs matplotlib.   usage: python3 sim/plot.py "caption line for each figure"
import re, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CAP = sys.argv[1] if len(sys.argv) > 1 else ""
uart = open("sim/out/uart.log", errors="replace").read().splitlines()
ang = [(float(m[1]), float(m[2])) for l in uart if (m := re.match(r"\[([\d.]+)s\] \(usb\) (-?[\d.]+)$", l))]
marks = [(float(m[1]), m[2]) for l in uart if (m := re.match(r"\[([\d.]+)s\] \(usb\) (MOT:\S.*|Motor ready!!!|STM32 Serial OK!)", l))]
plant = [tuple(map(float, re.findall(r"=(-?[\d.]+)", l.split("duty")[0])))
         + tuple(map(float, l.split("duty=")[1].split(","))) for l in open("sim/out/sim.log", errors="replace") if "PLANT t=" in l]
st, cmd = [], None
for l in open("sim/out/can-frames.txt", errors="replace"):
    m = re.match(r"\[dut\] \[([\d.]+)s\] \(can0\) CAN Tx id=0x123 dlc=8 data=(.*)", l)
    if m:
        (st if m[2].startswith("36 37 38 39 30") else [cmd := float(m[1])]).append(float(m[1])) if m[2].startswith("36 37") else (cmd := float(m[1]))

def fin(fig, name, ax):
    for a in (ax if hasattr(ax, "__iter__") else [ax]):
        a.grid(alpha=0.3)
        if cmd: a.axvline(cmd, color="tab:red", ls="--", lw=0.8)
    fig.suptitle(CAP, fontsize=7)
    fig.tight_layout()
    fig.savefig(f"sim/screenshots/{name}.png", dpi=90)

# 1. encoder angle as the firmware prints it, and the plant's shaft speed
fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
ax[0].plot(*zip(*ang[::20]), lw=0.8, label="encoder.getAngle() printed on the USB console (MT6835 over SPI1)")
ax[0].set_ylabel("rad"); ax[0].legend(loc="upper left", fontsize=7)
ax[1].plot([p[0] for p in plant], [p[1] for p in plant], color="tab:green", label="PMSM plant shaft speed")
ax[1].set_ylabel("rad/s"); ax[1].set_xlabel("virtual time (s); dashed = CAN command '32' at %.3f s" % cmd); ax[1].legend(loc="upper left", fontsize=7)
fin(fig, "encoder-and-speed", ax)

# 2. CAN 0x123: status frame interval, command
fig, ax = plt.subplots(figsize=(7, 3))
ax.plot(st[1:], [(b - a) * 1e3 for a, b in zip(st, st[1:])], ".", ms=3, label="joint status frame (\"67890\") interval, %d frames" % len(st))
ax.set_ylabel("ms"); ax.set_xlabel("virtual time (s); dashed = master's command frame"); ax.legend(fontsize=7)
fin(fig, "can-0x123", ax)

# 3. loop timing: gap between consecutive angle prints
fig, ax = plt.subplots(figsize=(7, 3))
ax.plot([t for t, _ in ang[1:]][::10], [(b[0] - a[0]) * 1e6 for a, b in zip(ang, ang[1:])][::10], ".", ms=2)
ax.set_ylabel("us between loop() prints"); ax.set_xlabel("virtual time (s)"); ax.set_ylim(0, 300)
fin(fig, "loop-timing", ax)

# 4. FOC state from the plant: dq currents and phase-A duty
fig, ax = plt.subplots(2, 1, figsize=(7, 4.5), sharex=True)
ax[0].plot([p[0] for p in plant], [p[2] for p in plant], label="iq"); ax[0].plot([p[0] for p in plant], [p[3] for p in plant], label="id")
ax[0].set_ylabel("A (plant model)"); ax[0].legend(fontsize=7)
ax[1].plot([p[0] for p in plant], [p[4] for p in plant], color="tab:purple", label="TIM4 CH1 duty (CCR1/ARR)")
ax[1].set_ylabel("duty"); ax[1].set_xlabel("virtual time (s)"); ax[1].legend(fontsize=7)
for t, s in marks:
    if s.startswith(("MOT:Align", "MOT:Zero", "Motor ready")): ax[1].annotate(s[:18], (t, 0.02), fontsize=6, rotation=90)
fin(fig, "foc-state", ax)
