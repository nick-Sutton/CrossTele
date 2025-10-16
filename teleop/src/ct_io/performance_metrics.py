from pose.pose import Pose
from typing import Dict, Tuple, Optional
import scipy.spatial.transform as st
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import csv
import yaml
import os
from datetime import datetime
import time

class PerformanceMetrics():
    def __init__(self, source_starting_pose=None, target_starting_pose=None, 
                 human = None, target_pose = None, source_twist = None, target_twist = None):
        self.logs_dir = "./teleop/log/"
        self.name = "log"
        self.dt = 0.004
        self.unit_scale = 1.0
        self.start_time = time.time()
        self.timestep_count = 0

        os.makedirs(self.logs_dir, exist_ok=True)
        # Generate unique filename with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        self.base_filename = f"{self.name}_{timestamp}"

        self.curr_run_dir = os.path.join(self.logs_dir, self.base_filename)
        os.makedirs(self.curr_run_dir, exist_ok=True)
        self.csv_path = os.path.join(self.curr_run_dir, f"{self.base_filename}.csv")
        self.yaml_path = os.path.join(self.curr_run_dir, f"{self.base_filename}_metadata.yaml")

        # Initialize CSV file with headers
        self._init_csv_file()
        
        # Store metadata
        self.metadata = {
            'experiment_name': self.name,
            'start_time': datetime.now().isoformat(),
            'log_directory': self.curr_run_dir,
            'csv_filename': os.path.basename(self.csv_path),
            'system_info': {
                'platform': os.uname().sysname if hasattr(os, 'uname') else 'Unknown',
                'python_version': os.sys.version
            }
        }

        self.source_starting_position = np.array([source_starting_pose["Root"].positionX / self.unit_scale, 
                                                    source_starting_pose["Root"].positionY / self.unit_scale,
                                                    source_starting_pose["Root"].positionZ / self.unit_scale])
        
        self.target_starting_position = np.array([target_starting_pose["Robot"].positionX, 
                                                  target_starting_pose["Robot"].positionY, 
                                                  target_starting_pose["Robot"].positionZ])

        self.human = human
        self.target_pose = target_pose
        self.source_twist = source_twist
        self.target_twist = target_twist

        # Key: timestep, value: metrics
        self.total_position_metrics = {}
        self.total_orientation_metrics = {}
        self.total_linear_velocity_metrics = {}
        self.total_angular_velocity_metrics = {}
        self.total_source_position = {}
        self.total_lfoot_position = {}
        self.total_rfoot_position = {}
        self.total_lfoot_lv = {}
        self.total_rfoot_lv = {}
        #self.gait_phase = {}

    def _init_csv_file(self):
        """Initialize CSV file with headers"""
        # Define all possible metric fields
        headers = ['timestep']
        
        # Position metrics
        position_fields = ['position_error', 'x_error', 'y_error', 'z_error']
        
        # Linear Velocity metrics  
        linear_velocity_fields = ['linear_velocity_error', 'x_vel_error', 'y_vel_error', 'z_vel_error']

        # Angular Velocity metrics  
        angular_velocity_fields = ['angular_velocity_error', 'rx_vel_error', 'ry_vel_error', 'rz_vel_error']
        
        # Add all fields to headers
        headers.extend(position_fields)
        headers.extend(linear_velocity_fields)
        headers.extend(angular_velocity_fields)
        
        # Write headers to CSV
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=headers)
            writer.writeheader()

    def position_metrics(self) -> Dict:
        curr_source_position = np.array([self.human.curr_pose["Root"].positionX / self.unit_scale, 
                                         self.human.curr_pose["Root"].positionY / self.unit_scale, 
                                         self.human.curr_pose["Root"].positionZ / self.unit_scale])
        
        curr_target_position = np.array([self.target_pose["Robot"].positionX, 
                                         self.target_pose["Robot"].positionY, 
                                         self.target_pose["Robot"].positionZ])

        source_displacement = curr_source_position - self.source_starting_position
        target_displacement = curr_target_position - self.target_starting_position

        pos_diff = source_displacement - target_displacement
        pos_error = np.linalg.norm(pos_diff) # Euclidean distance

        metrics = {
            'timestep': self.human.curr_pose["Root"].timestep,
            'position_error': pos_error,  # Euclidean distance
            'x_error': pos_diff[0],       # Error in X axis
            'y_error': pos_diff[1],       # Error in Y axis  
            'z_error': pos_diff[2],       # Error in Z axis
            'source_displacement': source_displacement,
            'target_displacement': target_displacement
        }

        self.total_position_metrics[self.human.curr_pose["Root"].timestep] = metrics
        self.total_source_position[self.human.curr_pose["Root"].timestep] = curr_source_position

        self.total_lfoot_position[self.human.curr_pose["LFoot"].timestep] = np.array([self.human.curr_pose["LFoot"].positionX / self.unit_scale, 
                                                                                   self.human.curr_pose["LFoot"].positionY / self.unit_scale, 
                                                                                   self.human.curr_pose["LFoot"].positionZ / self.unit_scale])
        
        self.total_rfoot_position[self.human.curr_pose["RFoot"].timestep] = np.array([self.human.curr_pose["RFoot"].positionX / self.unit_scale, 
                                                                                   self.human.curr_pose["RFoot"].positionY / self.unit_scale, 
                                                                                   self.human.curr_pose["RFoot"].positionZ / self.unit_scale])

        return metrics
    
    def orientation_metrics(self) -> Dict:
        # Source orientation as a quaturion
        source_q = st.Rotation.from_quat([
            self.human.curr_twist["Root"].orientationX,
            self.human.curr_twist["Root"].orientationY,
            self.human.curr_twist["Root"].orientationZ,
            self.human.curr_twist["Root"].orientationW
        ])
        
        # target orientation as a quaturion
        target_q = st.Rotation.from_quat([
            self.target_pose["Robot"].orientationX, 
            self.target_pose["Robot"].orientationY,
            self.target_pose["Robot"].orientationZ,
            self.target_pose["Robot"].orientationW
        ])

        # Calculate relative rotation
        q_diff = target_q.inv() * source_q
        
        # Get angle of rotation (in radians)
        angle_error_rad = q_diff.magnitude()

        # Convert to degrees
        angle_error_deg = np.degrees(angle_error_rad)


        metrics = {
            'angular_error_rad': angle_error_rad,
            'angular_error_deg': angle_error_deg
        }

        self.total_orientation_metrics[self.human.curr_twist["Root"].timestep] = metrics

        return metrics
    
    def linear_velocity_metrics(self):
        vel_diff = self.source_twist["Root"].linear_velocity - self.target_twist["Robot"].linear_velocity
        vel_error = np.linalg.norm(vel_diff)
        
        metrics = {
            'timestep': self.source_twist["Root"].timestep,
            'linear_velocity_error': vel_error,
            'x_vel_error': vel_diff[0],
            'y_vel_error': vel_diff[1],
            'z_vel_error': vel_diff[2],
            'source_linear_velocity': self.source_twist["Root"].linear_velocity,
            'target_linear_velocity': self.target_twist["Robot"].linear_velocity
        }

        self.total_linear_velocity_metrics[self.source_twist["Root"].timestep] = metrics
        self.total_lfoot_lv[self.source_twist["LFoot"].timestep] = self.source_twist["LFoot"].linear_velocity
        self.total_rfoot_lv[self.source_twist["RFoot"].timestep] = self.source_twist["RFoot"].linear_velocity
        #self.gait_phase[self.source_twist["RFoot"].timestep] = self.human.curr_gait_phase
        return metrics

    def angular_velocity_metrics(self):
        a_vel_diff = self.source_twist["Root"].angular_velocity - self.target_twist["Robot"].angular_velocity
        a_vel_error = np.linalg.norm(a_vel_diff)
        
        metrics = {
            'timestep': self.source_twist["Root"].timestep,
            'angular_velocity_error': a_vel_error,
            'rx_vel_error': a_vel_diff[0],
            'ry_vel_error': a_vel_diff[1],
            'rz_vel_error': a_vel_diff[2],
            'source_angular_velocity': self.source_twist["Root"].angular_velocity,
            'target_angular_velocity': self.target_twist["Robot"].angular_velocity
        }
        
        self.total_angular_velocity_metrics[self.source_twist["Root"].timestep] = metrics

        return metrics

    def temporal_accuracy(self):
        pass

    def response_time(self):
        pass

    def network_latency(self):
        pass

    def log_metrics(self):
        """Log metrics for a single timestep"""

        position_metrics = self.position_metrics()
        linear_velocity_metrics = self.linear_velocity_metrics()
        angular_velocity_metrics = self.angular_velocity_metrics()

        self.timestep_count += self.dt
        
        # Prepare data row
        row_data = {
            'timestep': position_metrics.get('timestep'),
            'position_error': round(position_metrics.get('position_error'), 4),
            'x_error': round(position_metrics.get('x_error'), 4),
            'y_error': round(position_metrics.get('y_error'), 4),
            'z_error': round(position_metrics.get('z_error'), 4),
            'linear_velocity_error': round(linear_velocity_metrics.get('linear_velocity_error'), 4),
            'x_vel_error': round(linear_velocity_metrics.get('x_vel_error'), 4),
            'y_vel_error': round(linear_velocity_metrics.get('y_vel_error'), 4),
            'z_vel_error': round(linear_velocity_metrics.get('z_vel_error'), 4),
            'angular_velocity_error': round(angular_velocity_metrics.get('angular_velocity_error'), 4),
            'rx_vel_error': round(angular_velocity_metrics.get('rx_vel_error'), 4),
            'ry_vel_error': round(angular_velocity_metrics.get('ry_vel_error'), 4),
            'rz_vel_error': round(angular_velocity_metrics.get('rz_vel_error'), 4)
        }
        
        # Write to CSV
        with open(self.csv_path, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=row_data.keys())
            writer.writerow(row_data)

    def print_metric_summary(self):
        """Print all metrics """
        
        # Define all metric categories with their display names and formats
        metric_categories = [
            {
                'name': 'POSITION',
                'data': self.position_metrics(),
                'fields': [
                    ('position_error', 'Total Error', '{:.4f} m'),
                    ('x_error', 'X Error', '{:.4f} m'),
                    ('y_error', 'Y Error', '{:.4f} m'),
                    ('z_error', 'Z Error', '{:.4f} m')
                ]
            },
            {
                'name': 'LINEAR VELOCITY',
                'data': self.linear_velocity_metrics(),
                'fields': [
                    ('linear_velocity_error', 'Total Error', '{:.4f} m/s'),
                    ('x_vel_error', 'X Error', '{:.4f} m/s'),
                    ('y_vel_error', 'Y Error', '{:.4f} m/s'),
                    ('z_vel_error', 'Z Error', '{:.4f} m/s')
                ]
            },
            {
                'name': 'ANGULAR VELOCITY',
                'data': self.angular_velocity_metrics(),
                'fields': [
                    ('angular_velocity_error', 'Total Error', '{:.4f} m/s'),
                    ('rx_vel_error', 'X Error', '{:.4f} m/s'),
                    ('ry_vel_error', 'Y Error', '{:.4f} m/s'),
                    ('rz_vel_error', 'Z Error', '{:.4f} m/s')
                ]
            },
        ]
        
        # Get timestep from first metric category
        timestep = metric_categories[0]['data'].get('timestep', 'N/A')
        
        # Print header
        print(f"\n{'='*50}")
        print(f"TIMESTEP: {timestep}")
        print(f"{'='*50}")
        
        # Print each metric category
        for category in metric_categories:
            title_line = f" {category['name']} "
            dashes_each_side = (50 - len(title_line)) // 2
            print(f"{'-'*dashes_each_side}{title_line}{'-'*dashes_each_side}")
            
            for field_key, display_name, value_format in category['fields']:
                value = category['data'].get(field_key)
                if value is not None:
                    formatted_value = value_format.format(value)
                    print(f"{display_name:<20}: {formatted_value:>25}")
                else:
                    print(f"{display_name:<20}: {'N/A':>25}")
        
        print(f"\n{'='*50}")

    def plot_position_metrics(self):
        """
        Plot position metrics over time for quadrupedal robot and human operator comparison.
        
        Args:
            total_position_metrics: Dictionary with timestep keys and metrics values
            save_path: Optional path to save the plot
        """
        # Extract data from metrics dictionary
        timesteps = []
        position_errors = []
        x_errors = []
        y_errors = []
        z_errors = []
        source_displacements = []
        target_displacements = []
        
        # Sort by timestep to ensure proper time ordering
        sorted_metrics = sorted(self.total_position_metrics.items(), key=lambda x: x[0])
        
        for timestep, metrics in sorted_metrics:
            timesteps.append(timestep)
            position_errors.append(metrics['position_error'])
            x_errors.append(metrics['x_error'])
            y_errors.append(metrics['y_error'])
            z_errors.append(metrics['z_error'])
            source_displacements.append(metrics['source_displacement'])
            target_displacements.append(metrics['target_displacement'])
        
        # Convert to numpy arrays for easier manipulation
        timesteps = np.array(timesteps)
        source_displacements = np.array(source_displacements)
        target_displacements = np.array(target_displacements)
        
        # Create comprehensive plot
        fig = plt.figure(figsize=(15, 12))
        
        # Plot 1: Overall position error over time
        plt.subplot(3, 2, 1)
        plt.plot(timesteps, position_errors, 'r-', linewidth=2, label='Position Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.title('Total Position Error Over Time')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 2: Individual axis errors
        plt.subplot(3, 2, 2)
        plt.plot(timesteps, x_errors, 'r-', label='X Error', alpha=0.8)
        plt.plot(timesteps, y_errors, 'g-', label='Y Error', alpha=0.8)
        plt.plot(timesteps, z_errors, 'b-', label='Z Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.title('Position Error by Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 3: Source (Human) displacement
        plt.subplot(3, 2, 3)
        plt.plot(timesteps, source_displacements[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, source_displacements[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, source_displacements[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (m)')
        plt.title('Source (Human) Displacement')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 4: Target (Robot) displacement
        plt.subplot(3, 2, 4)
        plt.plot(timesteps, target_displacements[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, target_displacements[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, target_displacements[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (m)')
        plt.title('Target (Robot) Displacement')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 5: 3D trajectory comparison
        ax = plt.subplot(3, 2, 5, projection='3d')
        ax.plot(source_displacements[:, 0], source_displacements[:, 1], 
                source_displacements[:, 2], 'r-', label='Human', linewidth=2)
        ax.plot(target_displacements[:, 0], target_displacements[:, 1], 
                target_displacements[:, 2], 'b-', label='Robot', linewidth=2)
        ax.set_xlabel('X Displacement (m)')
        ax.set_ylabel('Y Displacement (m)')
        ax.set_zlabel('Z Displacement (m)')
        ax.set_title('3D Trajectory Comparison')
        ax.legend()
        
        plt.tight_layout()
        
        plt.savefig(os.path.join(self.curr_run_dir, "position_plots"), dpi=300, bbox_inches='tight')

    def plot_linear_velocity_metrics(self):
        # Extract data from metrics dictionary
        timesteps = []
        linear_velocity_error = []
        x_vel_error = []
        y_vel_error = []
        z_vel_error = []
        source_lv = []
        target_lv = []
        
        # Sort by timestep to ensure proper time ordering
        sorted_metrics = sorted(self.total_linear_velocity_metrics.items(), key=lambda x: x[0])
        
        for timestep, metrics in sorted_metrics:
            timesteps.append(timestep)
            linear_velocity_error.append(metrics['linear_velocity_error'])
            x_vel_error.append(metrics['x_vel_error'])
            y_vel_error.append(metrics['y_vel_error'])
            z_vel_error.append(metrics['z_vel_error'])
            source_lv.append(metrics['source_linear_velocity'])
            target_lv.append(metrics['target_linear_velocity'])
        
        # Convert to numpy arrays for easier manipulation
        timesteps = np.array(timesteps)
        source_lv = np.array(source_lv)
        target_lv = np.array(target_lv)
        
        # Create comprehensive plot
        fig = plt.figure(figsize=(15, 12))
        
        # Plot 1: Overall position error over time
        plt.subplot(3, 2, 1)
        plt.plot(timesteps, linear_velocity_error, 'r-', linewidth=2, label='Linear Velocity Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity Error (m/s)')
        plt.title('Total Linear Velocity Error Over Time')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 2: Source (Human) LV
        plt.subplot(3, 2, 2)
        plt.plot(timesteps, source_lv[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, source_lv[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, source_lv[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Source (Human) Linear Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 3: Target (Robot) LV
        plt.subplot(3, 2, 3)
        plt.plot(timesteps, target_lv[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, target_lv[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, target_lv[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Target (Robot) Linear Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 4: X-Axis error
        plt.subplot(3, 2, 4)
        plt.plot(timesteps, x_vel_error, 'r-', label='X Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity Error (m/s)')
        plt.title('Linear Velocity Error X-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 5: Y-Axis error
        plt.subplot(3, 2, 5)
        plt.plot(timesteps, y_vel_error, 'g-', label='Y Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity Error (m/s)')
        plt.title('Linear Velocity Error Y-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 6: Z-Axis error
        plt.subplot(3, 2, 6)
        plt.plot(timesteps, z_vel_error, 'b-', label='Z Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity Error (m/s)')
        plt.title('Linear Velocity Error Z-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        plt.tight_layout()
        
        plt.savefig(os.path.join(self.curr_run_dir, "linear_vel_plots"), dpi=300, bbox_inches='tight')

    def plot_angular_velocity_metrics(self):
        # Extract data from metrics dictionary
        timesteps = []
        angular_velocity_error = []
        rx_vel_error = []
        ry_vel_error = []
        rz_vel_error = []
        source_av = []
        target_av = []
        
        # Sort by timestep to ensure proper time ordering
        sorted_metrics = sorted(self.total_angular_velocity_metrics.items(), key=lambda x: x[0])
        
        for timestep, metrics in sorted_metrics:
            timesteps.append(timestep)
            angular_velocity_error.append(metrics['angular_velocity_error'])
            rx_vel_error.append(metrics['rx_vel_error'])
            ry_vel_error.append(metrics['ry_vel_error'])
            rz_vel_error.append(metrics['rz_vel_error'])
            source_av.append(metrics['source_angular_velocity'])
            target_av.append(metrics['target_angular_velocity'])
        
        # Convert to numpy arrays for easier manipulation
        timesteps = np.array(timesteps)
        source_av = np.array(source_av)
        target_av = np.array(target_av)
        
        # Create comprehensive plot
        fig = plt.figure(figsize=(15, 12))
        
        # Plot 1: Overall position error over time
        plt.subplot(3, 2, 1)
        plt.plot(timesteps, angular_velocity_error, 'r-', linewidth=2, label='Angular Velocity Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity Error (m/s)')
        plt.title('Total Angular Velocity Error Over Time')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 3: Source (Human) AV
        plt.subplot(3, 2, 2)
        plt.plot(timesteps, source_av[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, source_av[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, source_av[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (m/s)')
        plt.title('Source (Human) Angular Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 4: Target (Robot) AV
        plt.subplot(3, 2, 3)
        plt.plot(timesteps, target_av[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, target_av[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, target_av[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (m/s)')
        plt.title('Target (Robot) Angular Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Plot 2: X-Axis error
        plt.subplot(3, 2, 4)
        plt.plot(timesteps, rx_vel_error, 'r-', label='X Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity Error (m/s)')
        plt.title('Angular Velocity Error X-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 3: Y-Axis Error
        plt.subplot(3, 2, 5)
        plt.plot(timesteps, ry_vel_error, 'g-', label='Y Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity Error (m/s)')
        plt.title('Angular Velocity Error Y-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 4: Z-Axis Error
        plt.subplot(3, 2, 6)
        plt.plot(timesteps, ry_vel_error, 'b-', label='Z Error', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity Error (m/s)')
        plt.title('Angular Velocity Error Z-Axis')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        plt.tight_layout()
        
        plt.savefig(os.path.join(self.curr_run_dir, "angular_vel_plots"), dpi=300, bbox_inches='tight')

    def plot_gait_features(self):

        # Create comprehensive plot
        fig = plt.figure(figsize=(15, 12))
        
        timesteps = []
        base_x_positions = []
        base_y_positions = []
        l_foot_z_positions = []
        r_foot_z_positions = []
        l_foot_lv = []
        r_foot_lv = []
        #gait_phase = []

        
        # Sort by timestep to ensure proper time ordering
        sorted_base_positions = sorted(self.total_source_position.items(), key=lambda x: x[0])
        for timestep, position in sorted_base_positions:
            timesteps.append(timestep)
            base_x_positions.append(position[0])  # base X coordinate
            base_y_positions.append(position[1])  # base Y coordinate

        for _, lv in self.total_lfoot_lv.items():
            l_foot_lv.append(lv)

        for _, lv in self.total_rfoot_lv.items():
            r_foot_lv.append(lv)

        for _, position in self.total_rfoot_position.items():
            r_foot_z_positions.append(position[2])

        for _, position in self.total_lfoot_position.items():
            l_foot_z_positions.append(position[2])

        #for _, gp in self.gait_phase.items():
        #    gait_phase.append(gp)
        
        # Convert to numpy arrays for easier manipulation
        timesteps = np.array(timesteps)
        base_x_positions = np.array(base_x_positions)
        base_y_positions = np.array(base_y_positions)
        l_foot_z_positions = np.array(l_foot_z_positions)
        r_foot_z_positions = np.array(r_foot_z_positions)
        l_foot_lv = np.array(l_foot_lv)
        r_foot_lv = np.array(r_foot_lv)
        #gait_phase = np.array(gait_phase)
        
        # Plot 1: X and Y position of Base
        plt.subplot(4, 2, 1)
        plt.plot(base_x_positions, base_y_positions, 'r-', linewidth=2, label='Time (s)')
        plt.xlabel('Base Position (X)')
        plt.ylabel('Base Position (Y)')
        plt.title('Base Position(X, Y)')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 2: Left and right foot z position
        plt.subplot(4, 2, 2)
        plt.plot(timesteps, l_foot_z_positions, 'g-', linewidth=2, label='Left Z Position')
        plt.plot(timesteps, r_foot_z_positions, 'b-', linewidth=2, label='Right Z Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Z Position (m)')
        plt.title('Combined Foot Z positions')
        plt.grid(True, alpha=0.3)
        plt.legend()

        # Plot 3: Left foot z position
        plt.subplot(4, 2, 3)
        plt.plot(timesteps, l_foot_z_positions, 'g-', linewidth=2, label='Z Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Z Position (m)')
        plt.title('Left Foot Z Position')
        plt.grid(True, alpha=0.3)
        plt.legend()

        #Plot 4: Right foot z position
        plt.subplot(4, 2, 4)
        plt.plot(timesteps, r_foot_z_positions, 'b-', linewidth=2, label='Z Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Z Position (m)')
        plt.title('Right Foot Z Position')
        plt.grid(True, alpha=0.3)
        plt.legend()

        #Plot 5: Left foot velocities
        plt.subplot(4, 2, 5)
        plt.plot(timesteps, l_foot_lv[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, l_foot_lv[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, l_foot_lv[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Left Foot Linear Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()

        #Plot 6 Right foot Velocities
        plt.subplot(4, 2, 6)
        plt.plot(timesteps, r_foot_lv[:, 0], 'r-', label='X', alpha=0.8)
        plt.plot(timesteps, r_foot_lv[:, 1], 'g-', label='Y', alpha=0.8)
        plt.plot(timesteps, r_foot_lv[:, 2], 'b-', label='Z', alpha=0.8)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Right Foot Linear Velocity')
        plt.grid(True, alpha=0.3)
        plt.legend()

        #Plot 7 Right foot Velocities
        #plt.subplot(4, 2, 7)
        #plt.plot(timesteps, gait_phase, 'r-', alpha=0.8)
        #plt.xlabel('Time (s)')
        #plt.ylabel('Gait Phase')
        #plt.title('Gait Phase')
        #plt.grid(True, alpha=0.3)
        #plt.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.curr_run_dir, "gait_plots"), dpi=300, bbox_inches='tight')

    def plot_metrics(self):
        self.plot_position_metrics()
        #self.plot_orientation_metrics()
        self.plot_linear_velocity_metrics()
        self.plot_angular_velocity_metrics()
        self.plot_gait_features()