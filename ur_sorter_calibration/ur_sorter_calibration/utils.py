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
