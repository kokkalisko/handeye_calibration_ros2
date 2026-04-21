"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation as R

import numpy as np
import yaml

# Create a QoS profile for subscribing to /tf_static
qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class TransformPublisher(Node):

    def __init__(self):
        super().__init__('transform_publisher')
        
        self.declare_parameter('handeye_result_file_name', '')
        self.declare_parameter('base_link', '')
        self.declare_parameter('ee_link', '')
        self.declare_parameter('world_frame', '')
        self.declare_parameter('calculated_camera_optical_frame_name', '')

        self.handeye_result_file_name = self.get_parameter('handeye_result_file_name').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.ee_link = self.get_parameter('ee_link').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.calculated_camera_optical_frame_name = self.get_parameter('calculated_camera_optical_frame_name').get_parameter_value().string_value
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Load transformation from YAML file
        with open(self.handeye_result_file_name, 'r') as file_2:
            hand_eye_data = yaml.safe_load(file_2)
            if isinstance(hand_eye_data, list) and len(hand_eye_data) > 1:
                hand_eye_data = hand_eye_data[-1]
            elif isinstance(hand_eye_data, list):
                hand_eye_data = hand_eye_data[0]

        T = np.eye(4)
        T[:3, :3] = np.array(hand_eye_data['rotation']).reshape((3, 3))
        T[:3, 3] = np.array(hand_eye_data['translation']).reshape((3,))

        self.translation = np.array(hand_eye_data['translation']).reshape((3, 1))
        self.rotation = np.array(hand_eye_data['rotation']).reshape((3, 3))

        r = R.from_matrix(self.rotation)
        self.handeye_quaternion = r.as_quat() #xyzw

        print(f'rotation: {self.rotation}')
        print(f'translation: {self.translation}')
        print(f'handeye_quaternion: {self.handeye_quaternion}')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_handeye_transform)

    def publish_handeye_transform(self):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.ee_link
        transform_msg.child_frame_id = self.calculated_camera_optical_frame_name
        transform_msg.transform.translation.x = self.translation[0, 0]
        transform_msg.transform.translation.y = self.translation[1, 0]
        transform_msg.transform.translation.z = self.translation[2, 0]
        transform_msg.transform.rotation.x = self.handeye_quaternion[0]
        transform_msg.transform.rotation.y = self.handeye_quaternion[1]
        transform_msg.transform.rotation.z = self.handeye_quaternion[2]
        transform_msg.transform.rotation.w = self.handeye_quaternion[3]

        self.tf_broadcaster.sendTransform(transform_msg)


def main(args=None):
    rclpy.init(args=args)

    transform_publisher = TransformPublisher()

    rclpy.spin(transform_publisher)

    transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
