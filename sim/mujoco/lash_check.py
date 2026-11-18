# No firmware, gravity off: drive the rotor with a +torque pulse then a -torque pulse and check the relative angle
# (rotor - load) is held inside +-backlash/2 by MuJoCo's tendon limit, and uses the whole gap.
import sys, os, numpy as np, mujoco
sys.path.insert(0, os.path.dirname(__file__))
import arm_xml
for b in (0.0044, 0.0175):
    m = mujoco.MjModel.from_xml_string(arm_xml.make(backlash=b)); m.opt.gravity[:] = 0
    d = mujoco.MjData(m); r = m.joint("rotor_j").qposadr[0]; l = m.joint("load_j").qposadr[0]
    rel = []
    for k in range(int(1.0 / m.opt.timestep)):
        t = k * m.opt.timestep
        d.ctrl[0] = 0.05 if t < 0.25 else -0.05 if t < 0.6 else 0.0
        mujoco.mj_step(m, d); rel.append(d.qpos[r] - d.qpos[l])
    rel = np.array(rel); width = rel.max() - rel.min()
    print(f"backlash {b:.4f} rad: relative angle ranged {rel.min():+.5f} .. {rel.max():+.5f}, width {width:.5f} rad ({width/b*100:.1f}% of the gap)")
    assert 0.85 * b < width < 1.15 * b
