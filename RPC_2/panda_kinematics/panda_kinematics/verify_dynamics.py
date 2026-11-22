import mujoco
import numpy as np
from controller_node import compute_dynamics, Q_MIN, Q_MAX

model = mujoco.MjModel.from_xml_path("/home/victustrio/project_name/src/mujoco_ros2/model/panda.xml")
data = mujoco.MjData(model)

rng = np.random.default_rng(42)
q = rng.uniform(Q_MIN, Q_MAX)
dq = rng.uniform(-1.0, 1.0, 7)
ddq = rng.uniform(-2.0, 2.0, 7)

data.qpos[:7] = q
data.qvel[:7] = dq
data.qacc[:7] = ddq
mujoco.mj_inverse(model, data)

M, Cg = compute_dynamics(q, dq, ddq)
tau_analytical = M @ ddq + Cg
tau_mujoco = data.qfrc_inverse[:7]

print("Analytical:", np.round(tau_analytical, 4))
print("MuJoCo:    ", np.round(tau_mujoco, 4))
print("Max error: ", np.max(np.abs(tau_analytical - tau_mujoco)))