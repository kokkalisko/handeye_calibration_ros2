import subprocess
from datetime import datetime
from pathlib import Path

import yaml
from ur_sorter_calibration.utils import *
from ur_sorter_calibration.calibration_algorithms import *

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32

class MarkerReader(Node):

    def __init__(self):
        super().__init__('marker_reader')

        self.get_logger().info("Initializing MarkerReader node...")

        self.declare_parameter('sensor_station_calibration', False)
        self.sensor_station_calibration = self.get_parameter('sensor_station_calibration').get_parameter_value().bool_value

        # Declare and load parameters related to the marker detection and camera calibration
        self.declare_parameter('marker_dictionary_name', '')
        self.declare_parameter('marker_name', '')
        self.declare_parameter('marker_side_length', 0.1)
        self.declare_parameter('camera_calibration_parameters_filename', '')
        self.declare_parameter('sensor_station_calibration_parameters_filename', '')

        marker_dictionary_name = self.get_parameter('marker_dictionary_name').get_parameter_value().string_value
        self.marker_name = self.get_parameter('marker_name').get_parameter_value().string_value
        self.marker_side_length = self.get_parameter('marker_side_length').get_parameter_value().double_value
        self.camera_calibration_parameters_filename = self.get_parameter('camera_calibration_parameters_filename').get_parameter_value().string_value
        self.sensor_station_calibration_parameters_filename = self.get_parameter('sensor_station_calibration_parameters_filename').get_parameter_value().string_value

        # Declare and load parameters related to the frames used
        self.declare_parameter('flange_to_camera', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('camera_optical_frame', '')
        self.declare_parameter('flange_link', '')
        self.declare_parameter('base_link', 'base_link')

        flange_to_camera = self.get_parameter('flange_to_camera').get_parameter_value().double_array_value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').get_parameter_value().string_value
        self.flange_link = self.get_parameter('flange_link').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value

        # Declare and load parameters related to the tracking frame calibration
        self.declare_parameter('image_topic', '')
        self.declare_parameter('tracking_frame_file', '')
        self.declare_parameter('sensor_station_image_topic', '')
        self.declare_parameter('encoder_scale_factor', 43.18)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.tracking_frame_file = self.get_parameter('tracking_frame_file').get_parameter_value().string_value
        self.sensor_station_image_topic = self.get_parameter('sensor_station_image_topic').get_parameter_value().string_value
        self.encoder_scale_factor = self.get_parameter('encoder_scale_factor').get_parameter_value().double_value

        # Check that we have a valid marker
        if MARKER_DICT.get(marker_dictionary_name, None) is None:
            self.get_logger().error(f"Marker tag of '{marker_dictionary_name}' is not supported")
            return

        # Load the (onboard) camera parameters from the saved file
        cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
        self.onboard_camera_mtx = cv_file.getNode('K').mat()
        self.onboard_camera_dst = cv_file.getNode('D').mat()
        cv_file.release()

        # Load the marker dictionary
        self.get_logger().info(f"Detecting '{marker_dictionary_name}' marker.")
        self.this_marker_dictionary = cv2.aruco.getPredefinedDictionary(MARKER_DICT[marker_dictionary_name])
        self.this_marker_parameters = cv2.aruco.DetectorParameters()

        # Create the image subscriber
        self.image_subscriber = self.create_subscription(Image, self.image_topic, self.listener_callback, 10)

        # Create the second image subscriber if topic is provided
        if self.sensor_station_image_topic and self.sensor_station_calibration:
            self.second_image_subscriber = self.create_subscription(Image,
                                                                    self.sensor_station_image_topic,
                                                                    self.sensor_station_listener_callback, 10)
            cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
            self.sensor_camera_mtx = cv_file.getNode('K').mat()
            self.sensor_camera_dst = cv_file.getNode('D').mat()
            cv_file.release()

        else:
            self.second_image_subscriber = None

        # Create the encoder subscriber for synchronization (and synchronize with encoder reader)
        self.encoder_subscriber = self.create_subscription(Int32, 'encoder_count', self.encoder_callback, 10)

        # Create the publisher for keypress events (and synchronize with encoder reader)
        self.keypress_publisher = self.create_publisher(String, 'keypress_topic', 10)

        # Publish the static transform from the flange to the camera
        self.publish_static_flange_to_camera(flange_to_camera)

        # Create the transform broadcaster to publish the transform from the marker to the camera
        self.tfbroadcaster = TransformBroadcaster(self)

        # Create TF buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Create the output resource folder and open it in the desktop file browser
        self.image_folder = self.create_image_output_folder()
        self.image_filename_pattern = str(self.image_folder / 'image_{pose_count:04d}.png')
        self.marker_data_file = self.image_folder / 'marker_data.yaml'
        self.encoder_data_file = self.image_folder / 'encoder_data.yaml'

        # Store the latest marker transform and encoder count for synchronization
        self.pose_count = 0
        self.encoder_value = None

        self.marker_entries = []
        self.encoder_entries = []

        if self.tracking_frame_file:
            self.load_tracking_frame(self.tracking_frame_file)

    def publish_static_flange_to_camera(self, flange_to_camera):
        if len(flange_to_camera) != 7:
            self.get_logger().error(
                'Expected flange_to_camera parameter length 7: [x, y, z, qx, qy, qz, qw]'
            )
            return

        translation = np.array(flange_to_camera[0:3], dtype=float)
        quaternion = np.array(flange_to_camera[3:7], dtype=float)

        if not np.isfinite(translation).all() or not np.isfinite(quaternion).all():
            self.get_logger().error('flange_to_camera contains invalid numeric values')
            return

        norm = np.linalg.norm(quaternion)
        if norm == 0.0:
            self.get_logger().error('flange_to_camera quaternion has zero length')
            return

        quaternion = quaternion / norm

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = self.flange_link
        static_transform.child_frame_id = self.camera_optical_frame
        static_transform.transform.translation.x = float(translation[0])
        static_transform.transform.translation.y = float(translation[1])
        static_transform.transform.translation.z = float(translation[2])
        static_transform.transform.rotation.x = float(quaternion[0])
        static_transform.transform.rotation.y = float(quaternion[1])
        static_transform.transform.rotation.z = float(quaternion[2])
        static_transform.transform.rotation.w = float(quaternion[3])

        self.tf_static_broadcaster.sendTransform(static_transform)
        self.get_logger().info(
            f'Published static transform from {self.flange_link} to {self.camera_optical_frame}'
        )

    def load_tracking_frame(self, filepath):
        try:
            with open(filepath, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)
                if data and 'tracking_frame' in data:
                    frame_list = data['tracking_frame']
                    if len(frame_list) == 7:
                        trans = np.array(frame_list[0:3])
                        quat = np.array(frame_list[3:7])
                        self.tracking_frame = (trans, quat)
                        self.get_logger().info(f"Loaded tracking_frame from {filepath}: Trans: {trans}, Quat: {quat}")
                    else:
                        self.get_logger().error(f"Invalid tracking_frame length in {filepath}")
        except Exception as exc:
            self.get_logger().error(f"Failed to load tracking_frame from {filepath}: {exc}")

    def create_image_output_folder(self):
        package_root = Path(__file__).resolve().parents[1]
        resource_root = package_root / 'resource'
        output_folder = resource_root / datetime.now().strftime('marker_images_%Y%m%d_%H%M%S')

        try:
            output_folder.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f'Failed to create image folder "{output_folder}": {exc}')
            raise

        self.get_logger().info(f'Saving images to {output_folder}')
        return output_folder

    def open_image_folder(self):
        try:
            subprocess.run(['xdg-open', str(self.image_folder)], check=False)
            self.get_logger().info(f'Opened image folder {self.image_folder}')
        except Exception as exc:
            self.get_logger().warning(
                f'Could not open image folder {self.image_folder}: {exc}'
            )

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data)
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(current_frame, self.this_marker_dictionary, parameters=self.this_marker_parameters)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_side_length, self.onboard_camera_mtx, self.onboard_camera_dst)
            
            for i, marker_id in enumerate(marker_ids):
                # Create the coordinate transform
                t_marker_to_camera = TransformStamped()
                t_marker_to_camera.header.stamp = self.get_clock().now().to_msg()

                t_marker_to_camera.header.frame_id = self.camera_optical_frame
                t_marker_to_camera.child_frame_id = self.marker_name

                # Store the translation (i.e. position) information
                t_marker_to_camera.transform.translation.x = tvecs[i][0][0]
                t_marker_to_camera.transform.translation.y = tvecs[i][0][1]
                t_marker_to_camera.transform.translation.z = tvecs[i][0][2]

                quat = rvec_to_quat(rvecs[i])

                # Quaternion format
                t_marker_to_camera.transform.rotation.x = quat[0]
                t_marker_to_camera.transform.rotation.y = quat[1]
                t_marker_to_camera.transform.rotation.z = quat[2]
                t_marker_to_camera.transform.rotation.w = quat[3]

                # Draw the axes on the marker
                cv2.drawFrameAxes(current_frame, self.onboard_camera_mtx, self.onboard_camera_dst, rvecs[i], tvecs[i], 0.05)

                # Send the transform from marker to the camera
                self.tfbroadcaster.sendTransform(t_marker_to_camera)

        # Create a named window that can be resized
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        # Resize the window to the desired size, e.g., 700x500 pixels
        cv2.resizeWindow("camera", 700, 500)

        cv2.imshow("camera", current_frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            if marker_ids is not None and len(marker_ids) > 0:
                self.get_logger().info(f"Detected markers: {marker_ids} with onboard camera")

                image_filename = self.image_filename_pattern.format(pose_count=self.pose_count)
                marker_entry = self.save_marker_data(marker_ids, rvecs, tvecs, image_filename, 'onboard')

                if marker_entry is not None:
                    self.save_image(current_frame)
                    self.get_logger().info(f"Saved marker transform and yaml entry.")
                    
                    self.pose_count += 1

                    if self.sensor_station_calibration:
                        self.keypress_publisher.publish(String(data='q'))
                else:
                    self.get_logger().warning("Marker entry was None, skipping save and synchronization.")

        elif key == ord('c'):
            self.get_logger().info("Performing calibration with the last 2 datapoints...")
            onboard_entries = [e for e in self.marker_entries if e['camera_source'] == 'onboard']
            if len(onboard_entries) < 2:
                self.get_logger().info("Not enough data points for calibration.")
            else:

                if not self.sensor_station_calibration:
                    onboard_entries = [e for e in self.marker_entries if e['camera_source'] == 'onboard']
                    if len(onboard_entries) < 2:
                        self.get_logger().info("Not enough onboard camera data points for calibration.")
                    else:
                        marker_entry1 = onboard_entries[-2]
                        marker_entry2 = onboard_entries[-1]
                        shared_markers = self.match_markers(marker_entry1, marker_entry2)

                        if not shared_markers:
                            self.get_logger().info("No common markers found between the last two datapoints, cannot perform calibration.")
                        else:
                            trans, quat = tracking_frame_calibrate(shared_markers)
                            self.get_logger().info(f"Tracking frame calibration result:")
                            self.get_logger().info(f"Translation: {trans}")
                            self.get_logger().info(f"Quaternion: {quat}")
                            self.write_frame_yaml(trans, quat)
                else:

                    sensor_station_entries = [e for e in self.marker_entries if e['camera_source'] == 'sensor_station']
                    onboard_entries = [e for e in self.marker_entries if e['camera_source'] == 'onboard']

                    if len(sensor_station_entries) < 1:
                        self.get_logger().info("No sensor station data points available for calibration.")
                        return
                    sensor_camera_entry = sensor_station_entries[-1]

                    if len(onboard_entries) < 1:
                        self.get_logger().info("No onboard camera data points available for calibration.")
                        return
                    onboard_camera_entry = onboard_entries[-1]

                    if self.tracking_frame is None:
                        self.get_logger().info("Tracking frame data is required for sensor station hand-eye calibration.")
                        return

                    encoder_counts = [
                        sensor_camera_entry.get('encoder_count'),
                        onboard_camera_entry.get('encoder_count')
                    ]
                    if None in encoder_counts:
                        self.get_logger().info("Missing encoder values for sensor station or onboard marker entry.")
                        return

                    shared_markers = self.match_markers(sensor_camera_entry, onboard_camera_entry)

                    if not shared_markers:
                        self.get_logger().info("No common markers found between sensor station and onboard camera entries, cannot perform hand-eye calibration.")
                        return

                    trans, quat = sensor_station_hand_eye_calibrate(
                        shared_markers,
                        self.tracking_frame,
                        np.array(encoder_counts, dtype=float),
                        self.encoder_scale_factor
                    )
                    self.get_logger().info(f"Hand-eye calibration result:")
                    self.get_logger().info(f"Translation: {trans}")
                    self.get_logger().info(f"Quaternion: {quat}")
                    self.write_frame_yaml(trans, quat, frame_name='hand_eye_calibration_frame')

        elif key == ord('e'):
            self.get_logger().info("Ending program...")
            self.keypress_publisher.publish(String(data='e'))
            cv2.destroyAllWindows()  # Close all OpenCV windows
            rclpy.shutdown()  # Shutdown ROS client library for Python

    def sensor_station_listener_callback(self, data):
        try:

            current_frame = self.bridge.imgmsg_to_cv2(data)
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(current_frame, self.this_marker_dictionary, parameters=self.this_marker_parameters)

            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_side_length, self.sensor_camera_mtx, self.sensor_camera_dst)
                
                for i, marker_id in enumerate(marker_ids):
                    # Create the coordinate transform
                    t_marker_to_camera = TransformStamped()
                    t_marker_to_camera.header.stamp = self.get_clock().now().to_msg()

                    t_marker_to_camera.header.frame_id = self.camera_optical_frame
                    t_marker_to_camera.child_frame_id = self.marker_name

                    # Store the translation (i.e. position) information
                    t_marker_to_camera.transform.translation.x = tvecs[i][0][0]
                    t_marker_to_camera.transform.translation.y = tvecs[i][0][1]
                    t_marker_to_camera.transform.translation.z = tvecs[i][0][2]

                    quat = rvec_to_quat(rvecs[i])

                    # Quaternion format
                    t_marker_to_camera.transform.rotation.x = quat[0]
                    t_marker_to_camera.transform.rotation.y = quat[1]
                    t_marker_to_camera.transform.rotation.z = quat[2]
                    t_marker_to_camera.transform.rotation.w = quat[3]

                    # Draw the axes on the marker
                    cv2.drawFrameAxes(current_frame, self.sensor_camera_mtx, self.sensor_camera_dst, rvecs[i], tvecs[i], 0.05)

                    # Send the transform from marker to the camera
                    self.tfbroadcaster.sendTransform(t_marker_to_camera)

            cv2.namedWindow("sensor_station_camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("sensor_station_camera", 700, 500)
            cv2.imshow("sensor_station_camera", current_frame)
            key = cv2.waitKey(1)
            if key == ord('s'):
                if marker_ids is not None and len(marker_ids) > 0:
                    self.get_logger().info(f"Detected markers: {marker_ids} with static camera")

                    image_filename = self.image_filename_pattern.format(pose_count=self.pose_count)
                    marker_entry = self.save_marker_data(marker_ids, rvecs, tvecs, image_filename, 'sensor_station')

                    if marker_entry is not None:
                        self.save_image(current_frame)
                        self.get_logger().info(f"Saved marker transform and yaml entry.")
                        
                        self.pose_count += 1

                        if self.sensor_station_calibration:
                            self.keypress_publisher.publish(String(data='q'))
                    else:
                        self.get_logger().warning("Marker entry was None, skipping save and synchronization.")

        except Exception as e:
            self.get_logger().error(f"Error in second_listener_callback: {e}")

    def match_markers(self, marker_entry1, marker_entry2):
        if marker_entry1 is None or marker_entry2 is None:
            self.get_logger().warning('One or both marker entries are missing for calibration.')
            return []

        markers1 = {m['marker_id']: m for m in marker_entry1.get('markers', [])}
        markers2 = {m['marker_id']: m for m in marker_entry2.get('markers', [])}

        shared_ids = sorted(set(markers1.keys()) & set(markers2.keys()))
        if not shared_ids:
            return []

        matched_marker_data = []
        for marker_id in shared_ids:
            m1 = markers1[marker_id]
            m2 = markers2[marker_id]
            matched_marker_data.append({
                'marker_id': marker_id,
                'entry1': {
                    'pose': m1['pose'],
                    'encoder_count': marker_entry1.get('encoder_count'),
                },
                'entry2': {
                    'pose': m2['pose'],
                    'encoder_count': marker_entry2.get('encoder_count'),
                },
            })

        if matched_marker_data:
            self.get_logger().info(
                f"Calibration data prepared for markers {', '.join(str(m['marker_id']) for m in matched_marker_data)}"
            )
        else:
            self.get_logger().info("No common marker_id found between the two latest datapoints.")

        return matched_marker_data

    def write_frame_yaml(self, trans, quat, frame_name='tracking_frame'):
        frame_list = [
            float(trans[0]),
            float(trans[1]),
            float(trans[2]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ]
        results_frames_file = self.image_folder / 'results_frames.yaml'

        try:
            with open(results_frames_file, 'w') as yaml_file:
                yaml.safe_dump({frame_name: frame_list}, yaml_file, sort_keys=False)
            self.get_logger().info(f'Wrote {frame_name} to {results_frames_file}')
        except OSError as exc:
            self.get_logger().error(f'Failed to write {frame_name} YAML: {exc}')

    def encoder_callback(self, msg):
        self.get_logger().info(f"Received encoder value: {msg.data}")
        self.encoder_value = msg.data
        entry = {
            'timestamp': datetime.now().isoformat(),
            'pose_count': int(self.pose_count),
            'encoder_count': int(msg.data),
        }
        self.encoder_entries.append(entry)
        self.write_encoder_data_yaml()

        if msg.data == 0:
            self.get_logger().info("Encoder count is zero, something was wrong.")
        else:
            self.get_logger().info(f"Encoder count is non-zero: {msg.data}.")

    def save_marker_data(self, marker_ids, rvecs, tvecs, image_filename=None, camera_source='onboard'):
        if marker_ids is None or rvecs is None or tvecs is None:
            self.get_logger().warning('No marker pose data available to save.')
            return

        marker_list = []
        for i, marker_id in enumerate(marker_ids.flatten()):
            # Create PoseStamped for the marker in camera frame
            pose = PoseStamped()
            pose.header.frame_id = self.camera_optical_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(tvecs[i][0][0])
            pose.pose.position.y = float(tvecs[i][0][1])
            pose.pose.position.z = float(tvecs[i][0][2])
            quat = rvec_to_quat(rvecs[i])
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])

            # Transform to base_link frame
            transformed_pose, error = transform_pose_between_frames(pose, self.base_link, self.tf_buffer)
            if error:
                self.get_logger().error(f'Failed to transform marker {marker_id} to {self.base_link}: {error}')
                return None

            marker_list.append({
                'marker_id': int(marker_id),
                'pose': transformed_pose,
            })

        entry = {
            'pose_count': int(self.pose_count),
            'timestamp': datetime.now().isoformat(),
            'encoder_count': int(self.encoder_value) if self.encoder_value is not None else None,
            'image_file': image_filename,
            'marker_count': len(marker_list),
            'markers': marker_list,
            'camera_source': camera_source,
        }

        self.marker_entries.append(entry)
        self.write_marker_data_yaml()

        return entry

    def write_marker_data_yaml(self):
        try:
            with open(self.marker_data_file, 'w') as yaml_file:
                yaml.safe_dump(
                    make_yaml_serializable({'marker_entries': self.marker_entries}),
                    yaml_file,
                    sort_keys=False
                )
            self.get_logger().info(f'Wrote marker pose data to {self.marker_data_file}')
        except Exception as exc:
            self.get_logger().error(f'Failed to write marker data YAML: {exc}')

    def write_encoder_data_yaml(self):
        try:
            with open(self.encoder_data_file, 'w') as yaml_file:
                yaml.safe_dump({'encoder_entries': self.encoder_entries}, yaml_file, sort_keys=False)
            self.get_logger().info(f'Wrote encoder data to {self.encoder_data_file}')
        except OSError as exc:
            self.get_logger().error(f'Failed to write encoder data YAML: {exc}')

    def save_image(self, image):
        image_filename = self.image_filename_pattern.format(pose_count=self.pose_count)
        if cv2.imwrite(image_filename, image):
            self.get_logger().info(f'Image saved as {image_filename}')
        else:
            self.get_logger().error(f'Failed to save image {image_filename}')

def main(args=None):
    rclpy.init(args=args)

    marker_reader = MarkerReader()

    rclpy.spin(marker_reader)

    marker_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
