#!/usr/bin/env python3
import os, sys, numpy as np, mujoco, glob

# Dynamically find the files
WS = os.path.expanduser("~/project_name")
sys.path.insert(0, os.path.dirname(glob.glob(os.path.join(WS, "src/**/kinematics.py"), recursive=True)[0]))
import kinematics as K

xml_path = glob.glob(os.path.join(WS, "src/**/franka_emika_panda/panda.xml"), recursive=True)[0]
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
hand = model.body("hand").id

rng = np.random.default_rng(0)
worst_pos = worst_ang = 0.0

for _ in range(200):
    q = rng.uniform(K.Q_MIN, K.Q_MAX)
    data.qpos[:7] = q
    mujoco.mj_forward(model, data)
    
    T_true = np.eye(4)
    T_true[:3, :3] = data.xmat[hand].reshape(3, 3)
    T_true[:3, 3] = data.xpos[hand]
    
    T_calc = K.forward_kinematics(q)
    pos = np.linalg.norm(T_true[:3, 3] - T_calc[:3, 3])
    c = np.clip((np.trace(T_true[:3, :3].T @ T_calc[:3, :3]) - 1) / 2, -1, 1)
    ang = np.degrees(np.arccos(c))
    
    worst_pos = max(worst_pos, pos)
    worst_ang = max(worst_ang, ang)

print("Compared 200 random configurations against MuJoCo.")
print(f"Worst position error:    {worst_pos * 1000:.4f} mm")
print(f"Worst orientation error: {worst_ang:.4f} deg")
if worst_pos < 1e-3 and worst_ang < 0.1:
    print("FK MATCHES the simulator.")
else:
    print("FK does NOT match.")
