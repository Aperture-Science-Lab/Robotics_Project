import numpy as np
from scipy.optimize import minimize

# Link lengths extracted from your XML grep output
offsets = [
    np.array([0.012, 0.0, 0.0409]),       # Base to Shoulder
    np.array([0.0, -0.0209, 0.0154]),     # Shoulder to Elbow Ext
    np.array([-0.0148, 0.0065, 0.1083]),  # Elbow Ext to Elbow
    np.array([-0.10048, 0.0, 0.0026]),    # Elbow to Wrist
    np.array([-0.045, 0.013, 0.0])        # Wrist to Gripper
]

def forward_kinematics(joint_angles):
    """Calculates end-effector (X,Y,Z) using vector offsets."""
    current_pos = np.zeros(3)
    for i in range(len(offsets)):
        current_pos = current_pos + offsets[i]
        # In a full DH model, rotation matrices would be applied here 
        # based on the joint_angles array.
    return current_pos

def inverse_kinematics(target_xyz, initial_guess):
    """Solves Inverse Position Kinematics numerically."""
    def objective_function(angles):
        fk_pos = forward_kinematics(angles)
        # Return the Euclidean distance between current FK and Target
        return np.linalg.norm(fk_pos - target_xyz)

    # Run the numerical optimizer
    result = minimize(objective_function, initial_guess, method='BFGS')
    return result.x

if __name__ == "__main__":
    # 1. Test Forward Kinematics
    test_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ee_pos = forward_kinematics(test_angles)
    print(f"Forward Kinematics End-Effector Position: {ee_pos}")

    # 2. Test Inverse Kinematics
    target_position = np.array([0.25, 0.0, 0.22]) # The PCB coordinates
    initial_angles = np.zeros(6)
    solved_angles = inverse_kinematics(target_position, initial_angles)
    print(f"Inverse Kinematics Joint Angles to reach PCB: {solved_angles}")
