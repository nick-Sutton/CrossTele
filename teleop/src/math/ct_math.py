import numpy as np
from scipy.spatial.transform import Rotation

def linear_velocity(current_pose, previous_pose, dt):
    """Calculate linear velocity from position change"""
    dx = current_pose.positionX - previous_pose.positionX
    dy = current_pose.positionY - previous_pose.positionY
    dz = current_pose.positionZ - previous_pose.positionZ
    
    linear_vel = np.array([dx/dt, dy/dt, dz/dt])
    return linear_vel

def angular_velocity(current_pose, previous_pose, dt):
    # Current rotation
    rot_current = Rotation.from_quat([
        current_pose.orientationX,
        current_pose.orientationY,
        current_pose.orientationZ,
        current_pose.orientationW
    ])
    
    # Previous rotation
    rot_previous = Rotation.from_quat([
        previous_pose.orientationX,
        previous_pose.orientationY,
        previous_pose.orientationZ,
        previous_pose.orientationW
    ])
    
    # Relative rotation
    rot_rel = rot_current * rot_previous.inv()
    
    # Convert to rotation vector and divide by dt
    rot_vec = rot_rel.as_rotvec()
    angular_vel = rot_vec / dt
    
    return angular_vel

def twist(current_pose, previous_pose, dt):
    """Calculate twist matrix with proper coordinate frame"""
    lv_m = linear_velocity(current_pose, previous_pose, dt)
    av_m = angular_velocity(current_pose, previous_pose, dt)
    
    # Construct the twist matrix
    twist_matrix = np.zeros((4, 4))
    
    # Skew-symmetric angular velocity (top-left 3x3)
    twist_matrix[0, 1] = -av_m[2]  # -ω_z
    twist_matrix[0, 2] = av_m[1]   # +ω_y
    twist_matrix[1, 0] = av_m[2]   # +ω_z  
    twist_matrix[1, 2] = -av_m[0]  # -ω_x
    twist_matrix[2, 0] = -av_m[1]  # -ω_y
    twist_matrix[2, 1] = av_m[0]   # +ω_x
    
    # Linear velocity (top-right 3x1)
    twist_matrix[0:3, 3] = lv_m
    
    return twist_matrix

def convert_twist_cordinate_frame(data_twist, robot_orientation):
    """
    Convert twist from world frame to robot body frame
    
    Parameters:
    - data_twist: 4x4 twist matrix in world coordinates
    - robot_orientation: quaternion [x, y, z, w] representing robot's current orientation
    
    Returns:
    - body_twist: 4x4 twist matrix in robot body coordinates
    """
    
    # Extract linear and angular velocities from world twist
    v_world = data_twist[0:3, 3]  # Linear velocity in world frame
    ω_world = np.array([
        data_twist[2, 1],  # ω_x
        data_twist[0, 2],  # ω_y  
        data_twist[1, 0]   # ω_z
    ])
    
    # Get rotation matrix from robot orientation (world to body)
    rot = Rotation.from_quat(robot_orientation)
    R = rot.as_matrix()  # Rotation from world to body frame
    
    # Transform velocities to body frame
    v_body = R.T @ v_world  # Linear velocity in body frame
    ω_body = R.T @ ω_world  # Angular velocity in body frame
    
    # Construct body-frame twist matrix
    body_twist = np.zeros((4, 4))
    
    # Skew-symmetric angular velocity
    body_twist[0, 1] = -ω_body[2]
    body_twist[0, 2] = ω_body[1]
    body_twist[1, 0] = ω_body[2]
    body_twist[1, 2] = -ω_body[0]
    body_twist[2, 0] = -ω_body[1]
    body_twist[2, 1] = ω_body[0]
    
    # Linear velocity
    body_twist[0:3, 3] = v_body
    
    return body_twist


#def angular_velocity(current_pose, previous_pose, dt):
    """Calculate angular velocity from quaternion difference"""
    
    # Current quaternion (normalized)
    q2 = np.array([
        current_pose.orientationX,
        current_pose.orientationY,
        current_pose.orientationZ,
        current_pose.orientationW
    ])
    q2 = q2 / np.linalg.norm(q2)
    
    # Previous quaternion (normalized)
    q1 = np.array([
        previous_pose.orientationX,
        previous_pose.orientationY,
        previous_pose.orientationZ,
        previous_pose.orientationW
    ])
    q1 = q1 / np.linalg.norm(q1)
    
    # Calculate relative rotation q_rel = q2 * q1_conjugate
    q1_conj = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
    
    q_rel = np.array([
        q2[3]*q1_conj[0] + q2[0]*q1_conj[3] + q2[1]*q1_conj[2] - q2[2]*q1_conj[1],
        q2[3]*q1_conj[1] - q2[0]*q1_conj[2] + q2[1]*q1_conj[3] + q2[2]*q1_conj[0],
        q2[3]*q1_conj[2] + q2[0]*q1_conj[1] - q2[1]*q1_conj[0] + q2[2]*q1_conj[3],
        q2[3]*q1_conj[3] - q2[0]*q1_conj[0] - q2[1]*q1_conj[1] - q2[2]*q1_conj[2]
    ])
    
    # Normalize relative quaternion
    q_rel = q_rel / np.linalg.norm(q_rel)
    
    # For small rotations, angular velocity ≈ (2/dt) * [qx, qy, qz]
    if q_rel[3] < 0:
        q_rel = -q_rel  # Ensure positive scalar part
    
    # Angular velocity calculation
    angle = 2 * np.arccos(np.clip(q_rel[3], -1.0, 1.0))
    if angle > np.pi:
        angle = 2 * np.pi - angle
    
    if angle < 1e-6:
        angular_vel = np.zeros(3)
    else:
        axis = q_rel[:3] / np.sin(angle/2)
        angular_vel = (angle / dt) * axis
    
    return angular_vel