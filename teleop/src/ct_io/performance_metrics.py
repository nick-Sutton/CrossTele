from pose.pose import Pose
from typing import Dict, Tuple, Optional
import scipy.spatial.transform as st
import numpy as np
import csv
import yaml
import os
from datetime import datetime
import time

class PerformanceMetrics():
    def __init__(self, source_starting_pose=None, target_starting_pose=None, 
                 source_pose = None, target_pose = None, source_twist = None, target_twist = None):
        self.log_dir = "./teleop/log"
        self.name = "log"
        self.dt = 0.004
        self.unit_scale = 1000.0
        self.start_time = time.time()
        self.timestep_count = 0

        os.makedirs(self.log_dir, exist_ok=True)
        # Generate unique filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base_filename = f"{self.name}_{timestamp}"
        self.csv_path = os.path.join(self.log_dir, f"{self.base_filename}.csv")
        self.yaml_path = os.path.join(self.log_dir, f"{self.base_filename}_metadata.yaml")

        # Initialize CSV file with headers
        self._init_csv_file()
        
        # Store metadata
        self.metadata = {
            'experiment_name': self.name,
            'start_time': datetime.now().isoformat(),
            'log_directory': self.log_dir,
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

        self.source_pose = source_pose
        self.target_pose = target_pose
        self.source_twist = source_twist
        self.target_twist = target_twist

        # Key: timestep, value: metrics
        self.total_position_metrics = {}
        self.total_orientation_metrics = {}
        self.total_linear_velocity_metrics = {}

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
        curr_source_position = np.array([self.source_pose["Root"].positionX / self.unit_scale, 
                                         self.source_pose["Root"].positionY / self.unit_scale, 
                                         self.source_pose["Root"].positionZ / self.unit_scale])
        
        curr_target_position = np.array([self.target_pose["Robot"].positionX, 
                                         self.target_pose["Robot"].positionY, 
                                         self.target_pose["Robot"].positionZ])

        source_displacement = curr_source_position - self.source_starting_position
        target_displacement = curr_target_position - self.target_starting_position

        pos_diff = source_displacement - target_displacement
        pos_error = np.linalg.norm(pos_diff) # Euclidean distance

        metrics = {
            'timestep': self.source_pose["Root"].timestep,
            'position_error': pos_error,  # Euclidean distance
            'x_error': pos_diff[0],       # Error in X axis
            'y_error': pos_diff[1],       # Error in Y axis  
            'z_error': pos_diff[2],       # Error in Z axis
            'source_displacement': source_displacement,
            'target_displacement': target_displacement
        }

        self.total_position_metrics[self.source_pose["Root"].timestep] = metrics

        return metrics
    
    def orientation_metrics(self) -> Dict:
        # Source orientation as a quaturion
        source_q = st.Rotation.from_quat([
            self.source_pose["Root"].orientationX,
            self.source_pose["Root"].orientationY,
            self.source_pose["Root"].orientationZ,
            self.source_pose["Root"].orientationW
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

        self.total_orientation_metrics[self.source_pose["Root"].timestep] = metrics

        return metrics
    
    def linear_velocity_metrics(self):
        vel_diff = self.source_twist["Root"].linear_velocity - self.target_twist["Robot"].linear_velocity
        vel_error = np.linalg.norm(vel_diff)
        
        metrics = {
            'timestep': self.source_twist["Root"].timestep,
            'linear_velocity_error': vel_error,
            'x_vel_error': vel_diff[0],
            'y_vel_error': vel_diff[1],
            'z_vel_error': vel_diff[2]
        }
        
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
        }
        
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



        