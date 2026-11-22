import numpy as np
import matplotlib.pyplot as plt
from panda_kinematics.controller_node import forward_kinematics

def load_run(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t, q, q_des = d[:, 0], d[:, 1:8], d[:, 8:15]
    z_act = np.array([forward_kinematics(qi)[2, 3] for qi in q])
    z_des = np.array([forward_kinematics(qi)[2, 3] for qi in q_des])
    rmse = np.sqrt(np.mean((q_des - q) ** 2))
    return t, q, q_des, z_act, z_des, rmse

runs = {
    "Baseline (Model + FLC)":         load_run("baseline.csv"),
    "Model Only (No FLC)":            load_run("no_flc.csv"),
    "2 kg Payload (Untuned PD-FLC)":  load_run("payload_2kg.csv"),
    "2 kg Payload (Robust PI+PD FLC)":load_run("payload_2kg_robust.csv"),
}

print(f"{'Run':<33} | {'Joint RMSE (rad)':>16} | {'Max Z Err (cm)':>14} | {'Final Z Err (cm)':>16}")
print("-" * 88)
for name, (t, q, q_des, z_act, z_des, rmse) in runs.items():
    z_err_cm = (z_act - z_des) * 100.0
    print(f"{name:<33} | {rmse:16.4f} | {np.max(np.abs(z_err_cm)):14.2f} | {z_err_cm[-1]:16.2f}")

fig, axes = plt.subplots(2, 1, figsize=(9.5, 7), sharex=True)
t_ref, _, _, _, z_des_ref, _ = runs["Baseline (Model + FLC)"]
axes[0].plot(t_ref, z_des_ref, "k--", linewidth=2, label="Desired Z")

for name, (t, q, q_des, z_act, z_des, _) in runs.items():
    axes[0].plot(t, z_act, linewidth=1.8, label=name)
    axes[1].plot(t, (z_act - z_des) * 100.0, linewidth=1.8, label=name)

axes[0].set_ylabel("End-Effector Z (m)")
axes[0].set_title("RPC 3 Team 4: Trajectory Tracking & 2 kg Payload Adaptation")
axes[0].legend(fontsize=9)
axes[0].grid(True)

axes[1].axhline(0.0, color="k", linestyle="--", linewidth=1)
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Z Tracking Error (cm)\n(Negative = Sag)")
axes[1].legend(fontsize=9)
axes[1].grid(True)

plt.tight_layout()
plt.savefig("rpc3_comparison.png", dpi=150)
print("Saved rpc3_comparison.png")
