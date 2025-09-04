import numpy as np
import scipy.spatial.transform as st

def linear_velocity(current_pose, previous_pose, dt):
    """Calculate linear velocity from position change"""

    unitScale = 1000.0

    dx = (current_pose.positionX - previous_pose.positionX) / unitScale
    dy = (current_pose.positionY - previous_pose.positionY) / unitScale  
    dz = (current_pose.positionZ - previous_pose.positionZ) / unitScale
    
    linear_vel = np.array([dx/dt, dy/dt, dz/dt])
    return linear_vel

def angular_velocity(current_pose, previous_pose, dt):
    # Current rotation
    rot_current = st.Rotation.from_quat([
        current_pose.orientationX,
        current_pose.orientationY,
        current_pose.orientationZ,
        current_pose.orientationW
    ])
    
    # Previous rotation
    rot_previous = st.Rotation.from_quat([
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

def eular_to_quat(roll, pitch, yaw):
    return st.Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()

def transform_cordinate_frame(human_linear_vel, human_angular_vel, robot_orientation):
    """
    Convert human velocities to robot reference frame with axis remapping.
    """
    
    # Remap coordinate axes with corrected direction
    human_remapped = np.array([
        -human_linear_vel[1],  # Human -Y (backward) -> Robot X (forward)
        -human_linear_vel[0],  # Human -X (left) -> Robot Y (left)
        human_linear_vel[2]    # Human Z (up) -> Robot Z (up)
    ])
    
    human_av_remapped = np.array([
        -human_angular_vel[1],
        -human_angular_vel[0], 
        human_angular_vel[2]
    ])
    
    # Transform from world frame to robot's local frame
    R_world_to_robot = st.Rotation.from_quat(robot_orientation).inv()
    
    robot_linear_vel = R_world_to_robot.apply(human_remapped)
    robot_angular_vel = R_world_to_robot.apply(human_av_remapped)
    
    return robot_linear_vel, robot_angular_vel
