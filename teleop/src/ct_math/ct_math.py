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

def transform_cordinate_frame(source_linear_vel, source_angular_vel, target_orientation):
    """
    Convert source velocities to target reference frame with axis remapping.
    """
    
    # Remap coordinate axes with corrected direction
    #source_lv_remapped, source_av_remapped = remap_cordinate_system(source_linear_vel, source_angular_vel)
    
    # Transform from world frame to target's local frame
    R_world_to_target = st.Rotation.from_quat(target_orientation).inv()
    
    target_linear_vel = R_world_to_target.apply(source_linear_vel)
    target_angular_vel = R_world_to_target.apply(source_angular_vel)
    
    return target_linear_vel, target_angular_vel

def apply_coordinate_transformation(df):
    """
    Apply coordinate system transformation to a motion capture DataFrame
    
    Args:
        df: DataFrame with motion capture data (formatted with RigidBodyName:Type:Axis columns)
        transform_coords: Whether to apply the coordinate transformation
    
    Returns:
        DataFrame with transformed coordinates (if transform_coords=True)
    """
    df_transformed = df.copy()
    
    # Find all unique rigid body names
    rigid_body_names = set()
    for col in df.columns:
        if ':' in col:
            parts = col.split(':')
            if len(parts) >= 2:
                rigid_body_names.add(parts[0])
    
    print(f"Applying coordinate transformation to {len(rigid_body_names)} rigid bodies...")
    
    for rb_name in rigid_body_names:
        # Transform Position data
        pos_x_col = f"{rb_name}:Position:X"
        pos_y_col = f"{rb_name}:Position:Y"
        pos_z_col = f"{rb_name}:Position:Z"
        
        if all(col in df.columns for col in [pos_x_col, pos_y_col, pos_z_col]):
            # Store original values
            orig_x = df[pos_x_col].values.copy()
            orig_y = df[pos_y_col].values.copy()
            orig_z = df[pos_z_col].values.copy()
            
            # Apply transformation: [x, y, z] -> [-y, -x, z]
            df_transformed[pos_x_col] = -orig_y  # New X = -original Y
            df_transformed[pos_y_col] = -orig_x  # New Y = -original X
            df_transformed[pos_z_col] = orig_z   # New Z = original Z
        
        # Transform Rotation data (quaternions)
        rot_x_col = f"{rb_name}:Rotation:X"
        rot_y_col = f"{rb_name}:Rotation:Y"
        rot_z_col = f"{rb_name}:Rotation:Z"
        rot_w_col = f"{rb_name}:Rotation:W"
        
        if all(col in df.columns for col in [rot_x_col, rot_y_col, rot_z_col, rot_w_col]):
            # Store original quaternion components
            orig_qx = df[rot_x_col].values.copy()
            orig_qy = df[rot_y_col].values.copy()
            orig_qz = df[rot_z_col].values.copy()
            orig_qw = df[rot_w_col].values.copy()
            
            # Apply same transformation to quaternion components
            df_transformed[rot_x_col] = -orig_qy  # New qx = -original qy
            df_transformed[rot_y_col] = -orig_qx  # New qy = -original qx
            df_transformed[rot_z_col] = orig_qz   # New qz = original qz
            df_transformed[rot_w_col] = orig_qw   # qw unchanged
    
    return df_transformed
