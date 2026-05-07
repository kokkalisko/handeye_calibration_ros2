import tf2_ros
from rclpy.duration import Duration
from geometry_msgs.msg import Quaternion

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

def transform_pose_between_frames(pose, target_frame, tf_buffer, timeout=1.0):
    try:
        # ---------------------------------------------------
        # Wait until transform is available
        # ---------------------------------------------------

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

def construct_frame(pA, RA, pB):
    # Desired X direction
    x = normalize(pB - pA)

    # Z axis from frame A
    z = normalize(RA[:, 2])

    # Orthogonal Y
    y = normalize(np.cross(z, x))

    # Recompute orthogonal X
    x = np.cross(y, z)

    # Rotation matrix
    R = np.column_stack((x, y, z))

    # Quaternion    
    r = Rotation.from_matrix(R)
    quat = r.as_quat()  # [x

    return pA, quat

def quaternion_to_matrix(q: Quaternion):
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return rot.as_matrix()
