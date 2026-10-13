#!/usr/bin/env python3
"""
Checks for the RPC Challenge 1 kinematics node. Called by rpc.sh.
"""
import os, re, subprocess, sys, time, glob
import numpy as np

WS = os.environ.get("RPC_WS", os.path.expanduser("~/project_name"))

# Dynamically find kinematics.py anywhere in the src folder
kin_paths = glob.glob(os.path.join(WS, "src/**/kinematics.py"), recursive=True)
if kin_paths:
    sys.path.insert(0, os.path.dirname(kin_paths[0]))
else:
    print("Error: Could not find kinematics.py in your src directory!")
    sys.exit(1)

import kinematics as K  # noqa: E402

XML = os.path.join(WS, "src/franka_emika_panda/panda.xml")
HOME = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
SIM_POS_MM, SIM_ANG_DEG = 5.0, 1.0

def pose_err(A, B):
    pos = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
    c = np.clip((np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2, -1, 1)
    return pos, np.degrees(np.arccos(c))

def load_mujoco():
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(XML)
        return mujoco, model, mujoco.MjData(model), model.body("hand").id
    except Exception as e:
        print(f"  (MuJoCo part skipped: {e})")
        return None

def mj_pose(mj, q):
    mujoco, model, data, hid = mj
    data.qpos[:7] = q
    mujoco.mj_forward(model, data)
    T = np.eye(4)
    T[:3, :3] = data.xmat[hid].reshape(3, 3)
    T[:3, 3] = data.xpos[hid]
    return T

def read_joints():
    try:
        out = subprocess.run(["ros2", "topic", "echo", "/joint_state", "--once"],
                             capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return None
    m = re.search(r"^position:\n((?:- .*\n)+)", out, re.M)
    if not m:
        return None
    return np.array([float(line[2:]) for line in m.group(1).splitlines()][:7])

def verdict(ok):
    return "PASS" if ok else "FAIL"

def cmd_check():
    q = read_joints()
    if q is None:
        print("No /joint_state received. Is the sim running?")
        return 1
    T = K.forward_kinematics(q)
    pos, ang = pose_err(T, K.TARGET_POSE)
    q_cmd, ok = K.inverse_kinematics(K.TARGET_POSE, q)
    ik_pos, ik_ang = pose_err(K.forward_kinematics(q_cmd), K.TARGET_POSE)
    print("Measured joints (rad):", np.round(q, 4))
    print(f"Hand vs TARGET_POSE in the sim : {pos:6.2f} mm, {ang:5.2f} deg")
    print(f"IK solution vs TARGET_POSE     : {ik_pos:6.4f} mm, {ik_ang:5.3f} deg (converged={ok})")
    print(f"Joint 5 = {q[4]:.3f} rad" + ("  <- near the singularity" if abs(q[4]) < 0.05 else ""))
    good = pos < SIM_POS_MM and ang < SIM_ANG_DEG
    print(f"\n{verdict(good)}: sim error under {SIM_POS_MM:g} mm and {SIM_ANG_DEG:g} deg.")
    return 0 if good else 1

def roundtrip(results, n=30):
    print(f"\nIK round trip ({n} random reachable targets): q -> FK -> IK -> FK")
    rng = np.random.default_rng(42)
    mj = load_mujoco()
    cases = (("near seed (true q + 0.3 rad noise)", lambda q: q + rng.normal(0, 0.3, 7)),
             ("far seed (home pose)", lambda q: HOME))
    for k, (label, seed_fn) in enumerate(cases):
        solved, worst_fk, worst_mj, times = 0, 0.0, 0.0, []
        for _ in range(n):
            q_true = rng.uniform(K.Q_MIN, K.Q_MAX)
            T = K.forward_kinematics(q_true)
            t0 = time.time()
            q_ik, ok = K.inverse_kinematics(T, np.clip(seed_fn(q_true), K.Q_MIN, K.Q_MAX))
            times.append(time.time() - t0)
            if not ok: continue
            solved += 1
            worst_fk = max(worst_fk, pose_err(K.forward_kinematics(q_ik), T)[0])
            if mj:
                worst_mj = max(worst_mj, pose_err(mj_pose(mj, q_ik), T)[0])
        extra = f", worst vs MuJoCo FK {worst_mj:.4f} mm" if mj else ""
        print(f"  {label}: solved {solved}/{n}, worst vs own FK {worst_fk:.4f} mm{extra}, "
              f"mean {1000 * np.mean(times):.0f} ms")
        if k == 0:
            results.append(("IK round trip, near seed: all solved, error < 0.1 mm",
                            solved == n and worst_fk < 0.1 and (not mj or worst_mj < 0.1)))

def cmd_roundtrip():
    results = []
    roundtrip(results)
    return 0 if all(ok for _, ok in results) else 1

def cmd_math():
    results = []
    err = lambda T, Tt: pose_err(T, Tt)[0]

    print("1) FK against MuJoCo (verify_fk.py)")
    vf = glob.glob(os.path.join(WS, "**/verify_fk.py"), recursive=True)
    if vf:
        out = subprocess.run([sys.executable, vf[0]], capture_output=True, text=True, cwd=WS); out = out.stdout + out.stderr
        print("  " + out.strip().replace("\n", "\n  "))
        results.append(("FK matches MuJoCo", "FK MATCHES" in out))
    else:
        print("  verify_fk.py not found in", WS)
        results.append(("FK matches MuJoCo", False))

    print("\n2) IK tests")
    seed = np.array([0, 0, 0, -1.57, 0, 1.57, 0.0])
    q2 = np.array([-0.8, 0.5, -0.6, -2.2, 0.4, 2.0, -0.3])
    T2 = K.forward_kinematics(q2)
    q, ok = K.inverse_kinematics(T2, seed)
    e = err(K.forward_kinematics(q), T2)
    print(f"  A) second target: ok={ok}, error={e:.4f} mm")
    results.append(("A) second target solved", ok and e < 0.1))

    Tu = K.TARGET_POSE.copy()
    Tu[:3, 3] = [2, 0, 1]
    t0 = time.time()
    q, ok = K.inverse_kinematics(Tu, seed)
    print(f"  B) unreachable target at (2, 0, 1): ok={ok}, {time.time() - t0:.2f} s")
    results.append(("B) unreachable target flagged False", not ok))

    qs = np.array([0.3, -0.4, 0.2, -1.8, 0.0, 1.6, 0.5])
    Ts = K.forward_kinematics(qs)
    try:
        q, ok = K.inverse_kinematics(Ts, qs + 0.05)
        e = err(K.forward_kinematics(q), Ts)
        print(f"  C) singularity q5=0: ok={ok}, error={e:.4f} mm, no crash")
        results.append(("C) singularity solved, no crash", ok and e < 0.1))
    except Exception as ex:
        print(f"  C) singularity CRASHED: {ex}")
        results.append(("C) singularity solved, no crash", False))

    q, _ = K.inverse_kinematics(K.TARGET_POSE, HOME)
    inside = bool(np.all(q >= K.Q_MIN) and np.all(q <= K.Q_MAX))
    print(f"  D) joint limits respected in the IK result: {inside}")
    results.append(("D) IK result inside joint limits", inside))
    roundtrip(results)

    print("\nSUMMARY")
    for name, ok in results:
        print(f"  {verdict(ok)}  {name}")
    return 0 if all(ok for _, ok in results) else 1

if __name__ == "__main__":
    cmds = {"check": cmd_check, "math": cmd_math, "roundtrip": cmd_roundtrip}
    if len(sys.argv) != 2 or sys.argv[1] not in cmds:
        sys.exit(2)
    sys.exit(cmds[sys.argv[1]]())
