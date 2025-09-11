import socket
import numpy as np
from ctypes import *

# UDP Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 8080  # Change this to match your MuJoCo setup

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Define data structure for mocap commands
class MocapCommand(Structure):
    _fields_ = [
        ("position", c_double * 3),    # x, y, z position
        ("orientation", c_double * 4), # quaternion: w, x, y, z
        ("timestamp", c_double)        # optional timestamp
    ]

def send_mocap_command(position, orientation):
    """
    Send position and orientation command to MuJoCo mocap object
    
    Args:
        position: [x, y, z] position coordinates
        orientation: [w, x, y, z] quaternion or [roll, pitch, yaw] euler angles
    """
    cmd = MocapCommand()
    
    # Set position
    for i in range(3):
        cmd.position[i] = position[i]
    
    # Handle orientation (assuming quaternion format)
    if len(orientation) == 4:
        # Quaternion format [w, x, y, z]
        for i in range(4):
            cmd.orientation[i] = orientation[i]
    elif len(orientation) == 3:
        # Convert from Euler angles to quaternion if needed
        # You'll need to implement this conversion based on your needs
        from scipy.spatial.transform import Rotation as R
        quat = R.from_euler('xyz', orientation).as_quat()
        # scipy returns [x, y, z, w], convert to [w, x, y, z]
        cmd.orientation[0] = quat[3]  # w
        cmd.orientation[1] = quat[0]  # x
        cmd.orientation[2] = quat[1]  # y
        cmd.orientation[3] = quat[2]  # z
    
    # Optional timestamp
    import time
    cmd.timestamp = time.time()
    
    # Send command
    try:
        sock.sendto(bytes(cmd), (UDP_IP, UDP_PORT))
        print(f"Sent mocap command: pos={position}, orient={orientation}")
    except Exception as e:
        print(f"Error sending command: {e}")

def set_mocap_position(x, y, z):
    """Set only position, keep current orientation"""
    send_mocap_command([x, y, z], [1.0, 0.0, 0.0, 0.0])  # Identity quaternion

def set_mocap_orientation(roll, pitch, yaw):
    """Set only orientation, keep current position at origin"""
    send_mocap_command([0.0, 0.0, 0.0], [roll, pitch, yaw])

def stop_mocap():
    """Send stop/home command"""
    send_mocap_command([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])

