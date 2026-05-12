import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time

import numpy as np
import cv2
import struct
from scipy.spatial.transform import Rotation

# ArUco and AprilTag dictionary lookup
MARKER_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def words_to_int32(low_word, high_word):
    """
    Combines two signed 16-bit integers (little-endian order)
    into a 32-bit unsigned integer.

    Args:
        low_word (int): Signed 16-bit integer (-32768 to 32767)
        high_word (int): Signed 16-bit integer (-32768 to 32767)

    Returns:
        int: 32-bit  integer (-2**31 <= result < 2**31-1)
    """
    # Pack the two 16-bit signed integers into 4 bytes (little-endian)
    packed = struct.pack('<hh', low_word, high_word)

    # Unpack as a 32-bit unsigned integer
    value, = struct.unpack('<i', packed)

    return value

def rvec_to_quat(rvec):
    # Store the rotation information
    rotation_matrix = cv2.Rodrigues(rvec[0])[0]
    r = Rotation.from_matrix(rotation_matrix)
    quat = r.as_quat()
    return quat

def transform_pose_between_frames(pose, target_frame, tf_buffer, timeout=5.0):
    try:
        # ---------------------------------------------------
        # Use the latest available transform to avoid small
        # timing skews and future-stamp extrapolation errors.
        # ---------------------------------------------------
        pose.header.stamp = Time(sec=0, nanosec=0)

        can_transform = tf_buffer.can_transform(
            target_frame,
            pose.header.frame_id,
            pose.header.stamp,
            timeout=Duration(seconds=timeout)
        )

        if not can_transform:
            return None, f'Transform from {pose.header.frame_id} to {target_frame} not available within {timeout} seconds.'
        
        # ---------------------------------------------------
        # Transform pose
        # ---------------------------------------------------

        transformed_pose = tf_buffer.transform(
            pose,
            target_frame,
            timeout=Duration(seconds=timeout)
        )

        return transformed_pose, None

    except tf2_ros.LookupException as e:
        return None, f'LookupException: {str(e)}'
    except tf2_ros.ConnectivityException as e:
        return None, f'ConnectivityException: {str(e)}'
    except tf2_ros.ExtrapolationException as e:
        return None, f'ExtrapolationException: {str(e)}'
    except Exception as e:
        return None, f'Unexpected exception: {str(e)}'
    
def normalize(v):
    return v / np.linalg.norm(v)

def quaternion_to_matrix(q: Quaternion):
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return rot.as_matrix()

def pose_stamped_to_list(pose_stamped):
    return [
        float(pose_stamped.pose.position.x),
        float(pose_stamped.pose.position.y),
        float(pose_stamped.pose.position.z),
        float(pose_stamped.pose.orientation.x),
        float(pose_stamped.pose.orientation.y),
        float(pose_stamped.pose.orientation.z),
        float(pose_stamped.pose.orientation.w)
    ]

def trans_list_to_matrix(trans_list):
    # Extract translation and quaternion from the list    
    translation = trans_list[:3]
    quat = trans_list[3:]

    # Convert quaternion to rotation matrix
    r = Rotation.from_quat(quat)
    R = r.as_matrix()

    # Construct the homogeneous transformation matrix
    trans_mat = np.eye(4)
    trans_mat[:3, :3] = R
    trans_mat[:3, 3] = translation

    return trans_mat

def make_yaml_serializable(obj):
    from geometry_msgs.msg import PoseStamped
    if isinstance(obj, dict):
        return {make_yaml_serializable(k): make_yaml_serializable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [make_yaml_serializable(v) for v in obj]
    if isinstance(obj, tuple):
        return [make_yaml_serializable(v) for v in obj]
    if isinstance(obj, PoseStamped):
        return pose_stamped_to_list(obj)
    if isinstance(obj, np.ndarray):
        return make_yaml_serializable(obj.tolist())
    if isinstance(obj, np.generic):
        return obj.item()
    return obj
