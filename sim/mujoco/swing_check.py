# No firmware: release the arm 0.1 rad from hanging, measure the swing period, compare with the
# physical-pendulum period 2*pi*sqrt(I / (m g d)) computed from MuJoCo's own mass and inertia.
import math, os, mujoco, numpy as np
m = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(__file__), "arm.xml"))
d = mujoco.MjData(m)
d.qpos[0] = 0.1
mujoco.mj_forward(m, d)
b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "link")
mass = m.body_subtreemass[b]
com = d.subtree_com[b]
dist = abs(com[2])
M = np.zeros((1, 1))
mujoco.mj_fullM(m, d, M)
I = M[0, 0]
T_ref = 2 * math.pi * math.sqrt(I / (mass * 9.81 * dist))
t, prev, crossings = 0.0, d.qpos[0], []
while len(crossings) < 5 and t < 10:
    mujoco.mj_step(m, d); t += m.opt.timestep
    if prev > 0 >= d.qpos[0]: crossings.append(t)
    prev = d.qpos[0]
T = (crossings[-1] - crossings[0]) / (len(crossings) - 1)
print(f"I={I:.5f} kg m^2  m={mass:.3f} kg  d={dist:.4f} m  analytic T={T_ref:.4f} s  simulated T={T:.4f} s  err={abs(T-T_ref)/T_ref*100:.2f}%")
assert abs(T - T_ref) / T_ref < 0.01
