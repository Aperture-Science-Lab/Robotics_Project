import numpy as np
import matplotlib.pyplot as plt
from panda_kinematics.controller_node import forward_kinematics

data = np.loadtxt("traj_log.csv", delimiter=",", skiprows=1)
t = data[:, 0]
q = data[:, 1:8]
q_des = data[:, 8:15]
err = q_des - q

# Compute end-effector Z height (useful for Team 4 payload sag later)
z_act = [forward_kinematics(qi)[2, 3] for qi in q]
z_des = [forward_kinematics(qi)[2, 3] for qi in q_des]

print("Max abs joint error (rad):", np.round(np.max(np.abs(err), axis=0), 4))
print("Final joint error (rad):  ", np.round(err[-1], 4))
print("Final Z error (cm):       ", round((z_des[-1] - z_act[-1]) * 100, 2))

fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for j in range(7):
    axes[0].plot(t, q_des[:, j], "--", alpha=0.6)
    axes[0].plot(t, q[:, j], label=f"J{j+1}")
axes[0].set_ylabel("Joint Angle (rad)")
axes[0].set_title("Desired (dashed) vs. Actual (solid) Joint Trajectory")
axes[0].legend(ncol=7, fontsize=8)
axes[0].grid(True)

axes[1].plot(t, z_des, "--", label="Desired Z")
axes[1].plot(t, z_act, label="Actual Z")
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("End-Effector Z (m)")
axes[1].legend()
axes[1].grid(True)

plt.tight_layout()
plt.savefig("tracking_plot.png", dpi=150)
print("Saved tracking_plot.png")