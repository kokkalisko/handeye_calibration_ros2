"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String

import yaml
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RobotTransformNode(Node):
    def __init__(self):
        super().__init__('robot_transform_node')
        self.pose_count = 0
        self.subscription_keypress = self.create_subscription(String, 'keypress_topic', self.keypress_callback, 10)
        
        self.declare_parameter('robot_data_file_name', '')
        self.declare_parameter('base_link', '')
        self.declare_parameter('ee_link', '')

        self.robot_data_file_name = self.get_parameter('robot_data_file_name').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.ee_link = self.get_parameter('ee_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """ Convert a quaternion into a full three-dimensional rotation matrix. """
        return R.from_quat([x, y, z, w]).as_matrix()

    def get_full_transformation_matrix(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.base_link,
                self.ee_link,
                now)
            
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            
            T = np.eye(4)
            T[:3, :3] = self.quaternion_to_rotation_matrix(*rotation)
            T[:3, 3] = translation
            
            return T
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_link} to {self.ee_link}: {ex}')
            return None

    def save_transformation_to_yaml(self, rotation_matrix, translation_vector):
        """ Append the rotation matrix and translation vector to a YAML file and print them. """
        # Load existing data from YAML if file exists and is not empty
        yaml_file_path = self.robot_data_file_name
        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file) or {'poses': []}  # Use existing data or initialize if empty
        except FileNotFoundError:
            data = {'poses': []}  # Initialize if file does not exist

        # Append new pose data
        data['poses'].append({
            'rotation': rotation_matrix.tolist(),
            'translation': translation_vector.tolist()
        })

        # Write updated data back to YAML
        with open(yaml_file_path, 'w') as file:  # 'w' to overwrite existing file
            yaml.dump(data, file, default_flow_style=False)

        self.pose_count += 1  # Increment pose counter
        print(f"Pose {self.pose_count}:")
        print("Rotation Matrix:")
        print(rotation_matrix)
        print("Translation Vector:")
        print(translation_vector)
        self.get_logger().info(f'Transformation for Pose {self.pose_count} appended to robot_data_realsense.yaml')
    
    def keypress_callback(self, msg):
        key = msg.data
        if key == 'q':
            T = self.get_full_transformation_matrix()
            if T is not None:
                R_gripper2base = T[:3, :3]
                t_gripper2base = T[:3, 3]
                self.save_transformation_to_yaml(R_gripper2base, t_gripper2base)
        elif key == 'e':
            self.get_logger().info("Ending program...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_transform_node = RobotTransformNode()

    try:
        rclpy.spin(robot_transform_node)
    finally:
        robot_transform_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
