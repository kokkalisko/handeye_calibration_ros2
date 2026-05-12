import numpy as np
from scipy.spatial.transform import Rotation
from ur_sorter_calibration.utils import *

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
    quat = r.as_quat()  # [x, y, z, w]

    return pA, quat

def tracking_frame_calibrate(matched_markers):
    # Select the first matched marker for calibration
    marker_data = matched_markers[0]
    marker_id = marker_data['marker_id']

    # Extract the relevant data for calibration
    first_point = marker_data['entry1']
    second_point = marker_data['entry2']

    pA = np.array([
        first_point['pose'].pose.position.x,
        first_point['pose'].pose.position.y,
        first_point['pose'].pose.position.z])
    RA = quaternion_to_matrix(first_point['pose'].pose.orientation)

    pB = np.array([
        second_point['pose'].pose.position.x,
        second_point['pose'].pose.position.y,
        second_point['pose'].pose.position.z])

    # Construct the frame
    trans, quat = construct_frame(pA, RA, pB)

    return trans, quat


def transform_poses_to_tracking_frame(robot_poses, robot_to_tracking_trans, encoder_values, scale_factor):
    """
    Transforms robot poses from the robot world to the tracking frame.

    Parameters:
    robot_poses (numpy.ndarray): A 2D numpy array representing the robot poses in the robot world. Each row corresponds to a pose, and the columns represent x, y, and z coordinates in meters.
    robot_to_tracking_trans (tuple): A tuple (trans, quat) representing the transformation from the robot world to the tracking frame, where trans is [tx, ty, tz] in meters, quat is [qx, qy, qz, qw].
    encoder_values (numpy.ndarray): A 1D numpy array representing the encoder values. The first two elements are used to calculate the translation distance.

    Returns:
    numpy.ndarray: A 2D numpy array representing the transformed robot poses in the tracking frame. Each row corresponds to a pose, and the columns represent x, y, and z coordinates in meters.
    """
    # Convert encoder values to meters
    translation_distance = (encoder_values[1] - encoder_values[0]) / scale_factor
    translation_distance_meters = translation_distance / 1000.0  # assuming scale_factor is counts per mm

    # robot_poses is already in meters
    robot_poses = np.transpose(robot_poses)  # (3, N)
    robot_poses = np.vstack([robot_poses, np.ones(robot_poses.shape[1])])  # (4, N)

    # Get the robot world to tracking frame transformation matrix
    trans, quat = robot_to_tracking_trans
    r = Rotation.from_quat(quat)
    R = r.as_matrix()
    robot_to_tracking_mat = np.eye(4)
    robot_to_tracking_mat[:3, :3] = R
    robot_to_tracking_mat[:3, 3] = trans

    # Inverse matrix to get the transformation from tracking to robot world
    transformation_mat = np.linalg.inv(robot_to_tracking_mat)
    transformation_mat[0:3, 3] = transformation_mat[0:3, 3] - np.array([translation_distance_meters, 0.0, 0.0])

    # Apply the transformation matrix to the robot poses
    transformed_poses = transformation_mat @ robot_poses
    return np.transpose(transformed_poses[:3, :])


def kabsch_algorithm(P, Q):
    """
    Compute the optimal rotation matrix and translation vector that aligns two sets of 3D points using the Kabsch algorithm.
    Parameters:
    P (numpy.ndarray): A 2D numpy array of shape (N, 3) representing the first set of 3D points.
    Q (numpy.ndarray): A 2D numpy array of shape (N, 3) representing the second set of 3D points.
    Returns:
    R (numpy.ndarray): A 2D numpy array of shape (3, 3) representing the optimal rotation matrix.
    t (numpy.ndarray): A 1D numpy array of shape (3,) representing the optimal translation vector.
    """
    C_P = np.mean(P, axis=0)
    C_Q = np.mean(Q, axis=0)
    
    P_centered = P - C_P
    Q_centered = Q - C_Q
    
    H = P_centered.T @ Q_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    t = -R @ C_P.T + C_Q.T

    return R, t

def calculate_rmsd(P, Q, R):
    """
    Calculate the Root-Mean-Square Deviation (RMSD) between two sets of 3D points after applying a transformation matrix.

    Parameters:
    P (numpy.ndarray): A 2D numpy array of shape (N, 3) representing the original set of 3D points.
    Q (numpy.ndarray): A 2D numpy array of shape (N, 3) representing the transformed set of 3D points.
    R (numpy.ndarray): A 2D numpy array of shape (3, 3) representing the rotation matrix applied to the original points.

    Returns:
    float: The RMSD value between the original and transformed points after applying the rotation matrix.
    """
    P_centered = P - np.mean(P, axis=0)
    Q_centered = Q - np.mean(Q, axis=0)
    
    P_rotated = P_centered @ R
    return np.sqrt(np.mean(np.sum((P_rotated - Q_centered) ** 2, axis=1)))

def transform_poses_to_camera_frame(matched_markers):
    # Implementation for transforming poses to camera frame
    pass

def sensor_station_hand_eye_calibrate(matched_markers, robot_to_tracking_trans, encoder_values, scale_factor):
    """
    Calibrate the hand-eye transformation for the sensor station using the Kabsch algorithm.

    matched_markers (list): A list of matched marker data where each element contains the
        camera and onboard marker poses that refer to the same marker ID.
    robot_to_tracking_trans (tuple): Tracking frame transform (trans, quat) from load_tracking_frame.
    encoder_values (numpy.ndarray): Encoder counts associated with the sensor and onboard marker entries.
    scale_factor (float): A scaling factor to convert encoder values into distance units.

    Returns:
    trans (numpy.ndarray): Translation vector [x, y, z] from camera frame to tracking frame.
    quat (numpy.ndarray): Quaternion [x, y, z, w] from camera frame to tracking frame.
    rmsd (float): The Root-Mean-Square Deviation (RMSD) between the original and transformed points after applying the rotation matrix.
    """

    # Extract camera poses (sensor station) and robot poses (onboard camera)
    camera_poses = [m['entry1']['pose'] for m in matched_markers]
    robot_poses = [m['entry2']['pose'] for m in matched_markers]

    # Convert to numpy arrays of positions (assuming meters)
    camera_positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in camera_poses])
    robot_positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in robot_poses])

    # Transform the robot poses to the tracking frame
    transformed_robot_positions = transform_poses_to_tracking_frame(robot_positions, robot_to_tracking_trans, encoder_values, scale_factor)

    # Deploy Kabsch algorithm to find the optimal rotation and translation
    R, t = kabsch_algorithm(transformed_robot_positions, camera_positions)

    # Calculate RMSD to evaluate the quality of the transformation
    rmsd = calculate_rmsd(camera_positions, transformed_robot_positions, R)

    # Convert rotation matrix to quaternion
    r = Rotation.from_matrix(R)
    quat = r.as_quat()  # [x, y, z, w]

    return t, quat, rmsd