"""
Transform Utilities

Common utilities for quaternion and vector operations used in robotics.
"""

import math
from typing import List, Tuple, Union
from geometry_msgs.msg import Vector3, Quaternion, Point, Pose


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q: Quaternion message with x, y, z, w components

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Convert quaternion to Euler angles using the ZYX convention
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.

    Args:
        roll: Roll angle in radians
        pitch: Pitch angle in radians
        yaw: Yaw angle in radians

    Returns:
        Quaternion message with x, y, z, w components
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """
    Normalize a quaternion to unit length.

    Args:
        q: Input quaternion

    Returns:
        Normalized quaternion
    """
    magnitude = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)

    if magnitude == 0:
        # Return identity quaternion if input is zero
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    return Quaternion(
        w=q.w / magnitude,
        x=q.x / magnitude,
        y=q.y / magnitude,
        z=q.z / magnitude
    )


def quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """
    Multiply two quaternions: q1 * q2.

    Args:
        q1: First quaternion
        q2: Second quaternion

    Returns:
        Result of quaternion multiplication
    """
    result = Quaternion()
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w

    return result


def quaternion_inverse(q: Quaternion) -> Quaternion:
    """
    Calculate the inverse of a quaternion.

    Args:
        q: Input quaternion

    Returns:
        Inverse quaternion
    """
    magnitude_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z

    if magnitude_sq == 0:
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    return Quaternion(
        w=q.w / magnitude_sq,
        x=-q.x / magnitude_sq,
        y=-q.y / magnitude_sq,
        z=-q.z / magnitude_sq
    )


def vector_magnitude(v: Vector3) -> float:
    """
    Calculate the magnitude of a vector.

    Args:
        v: Input vector

    Returns:
        Magnitude of the vector
    """
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def vector_normalize(v: Vector3) -> Vector3:
    """
    Normalize a vector to unit length.

    Args:
        v: Input vector

    Returns:
        Normalized vector
    """
    mag = vector_magnitude(v)

    if mag == 0:
        return Vector3(x=0.0, y=0.0, z=0.0)

    return Vector3(x=v.x / mag, y=v.y / mag, z=v.z / mag)


def vector_dot(v1: Vector3, v2: Vector3) -> float:
    """
    Calculate the dot product of two vectors.

    Args:
        v1: First vector
        v2: Second vector

    Returns:
        Dot product of the vectors
    """
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z


def vector_cross(v1: Vector3, v2: Vector3) -> Vector3:
    """
    Calculate the cross product of two vectors.

    Args:
        v1: First vector
        v2: Second vector

    Returns:
        Cross product of the vectors
    """
    return Vector3(
        x=v1.y * v2.z - v1.z * v2.y,
        y=v1.z * v2.x - v1.x * v2.z,
        z=v1.x * v2.y - v1.y * v2.x
    )


def transform_point(point: Point, translation: Vector3, rotation: Quaternion) -> Point:
    """
    Transform a point by a translation and rotation.

    Args:
        point: Input point to transform
        translation: Translation vector
        rotation: Rotation quaternion

    Returns:
        Transformed point
    """
    # Convert point to quaternion form for rotation
    p_quat = Quaternion(x=point.x, y=point.y, z=point.z, w=0.0)

    # Rotate the point: q * p * q_inverse
    rotated_quat = quaternion_multiply(
        quaternion_multiply(rotation, p_quat),
        quaternion_inverse(rotation)
    )

    # Add translation
    result = Point()
    result.x = rotated_quat.x + translation.x
    result.y = rotated_quat.y + translation.y
    result.z = rotated_quat.z + translation.z

    return result


def pose_to_transformation_matrix(pose: Pose) -> List[List[float]]:
    """
    Convert a pose to a 4x4 transformation matrix.

    Args:
        pose: Input pose with position and orientation

    Returns:
        4x4 transformation matrix as a list of lists
    """
    # Convert quaternion to rotation matrix
    q = pose.orientation
    rotation_matrix = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]

    # Calculate rotation matrix from quaternion
    rotation_matrix[0][0] = 1 - 2 * (q.y * q.y + q.z * q.z)
    rotation_matrix[0][1] = 2 * (q.x * q.y - q.w * q.z)
    rotation_matrix[0][2] = 2 * (q.x * q.z + q.w * q.y)
    rotation_matrix[1][0] = 2 * (q.x * q.y + q.w * q.z)
    rotation_matrix[1][1] = 1 - 2 * (q.x * q.x + q.z * q.z)
    rotation_matrix[1][2] = 2 * (q.y * q.z - q.w * q.x)
    rotation_matrix[2][0] = 2 * (q.x * q.z - q.w * q.y)
    rotation_matrix[2][1] = 2 * (q.y * q.z + q.w * q.x)
    rotation_matrix[2][2] = 1 - 2 * (q.x * q.x + q.y * q.y)

    # Create 4x4 transformation matrix
    matrix = [
        [1.0, 0.0, 0.0, pose.position.x],
        [0.0, 1.0, 0.0, pose.position.y],
        [0.0, 0.0, 1.0, pose.position.z],
        [0.0, 0.0, 0.0, 1.0]
    ]

    # Copy rotation components
    for i in range(3):
        for j in range(3):
            matrix[i][j] = rotation_matrix[i][j]

    return matrix


def transformation_matrix_to_pose(matrix: List[List[float]]) -> Pose:
    """
    Convert a 4x4 transformation matrix to a pose.

    Args:
        matrix: 4x4 transformation matrix

    Returns:
        Pose with position and orientation
    """
    pose = Pose()

    # Extract position
    pose.position.x = matrix[0][3]
    pose.position.y = matrix[1][3]
    pose.position.z = matrix[2][3]

    # Extract rotation matrix and convert to quaternion
    rot_matrix = [
        [matrix[0][0], matrix[0][1], matrix[0][2]],
        [matrix[1][0], matrix[1][1], matrix[1][2]],
        [matrix[2][0], matrix[2][1], matrix[2][2]]
    ]

    # Convert rotation matrix to quaternion
    trace = rot_matrix[0][0] + rot_matrix[1][1] + rot_matrix[2][2]

    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2  # S=4*qw
        qw = 0.25 * s
        qx = (rot_matrix[2][1] - rot_matrix[1][2]) / s
        qy = (rot_matrix[0][2] - rot_matrix[2][0]) / s
        qz = (rot_matrix[1][0] - rot_matrix[0][1]) / s
    else:
        if rot_matrix[0][0] > rot_matrix[1][1] and rot_matrix[0][0] > rot_matrix[2][2]:
            s = math.sqrt(1.0 + rot_matrix[0][0] - rot_matrix[1][1] - rot_matrix[2][2]) * 2
            qw = (rot_matrix[2][1] - rot_matrix[1][2]) / s
            qx = 0.25 * s
            qy = (rot_matrix[0][1] + rot_matrix[1][0]) / s
            qz = (rot_matrix[0][2] + rot_matrix[2][0]) / s
        elif rot_matrix[1][1] > rot_matrix[2][2]:
            s = math.sqrt(1.0 + rot_matrix[1][1] - rot_matrix[0][0] - rot_matrix[2][2]) * 2
            qw = (rot_matrix[0][2] - rot_matrix[2][0]) / s
            qx = (rot_matrix[0][1] + rot_matrix[1][0]) / s
            qy = 0.25 * s
            qz = (rot_matrix[1][2] + rot_matrix[2][1]) / s
        else:
            s = math.sqrt(1.0 + rot_matrix[2][2] - rot_matrix[0][0] - rot_matrix[1][1]) * 2
            qw = (rot_matrix[1][0] - rot_matrix[0][1]) / s
            qx = (rot_matrix[0][2] + rot_matrix[2][0]) / s
            qy = (rot_matrix[1][2] + rot_matrix[2][1]) / s
            qz = 0.25 * s

    pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

    return pose


def calculate_distance_3d(point1: Union[Point, Vector3], point2: Union[Point, Vector3]) -> float:
    """
    Calculate the 3D distance between two points.

    Args:
        point1: First point/vector
        point2: Second point/vector

    Returns:
        Distance between the points
    """
    dx = point2.x - point1.x
    dy = point2.y - point1.y
    dz = point2.z - point1.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def is_quaternion_normalized(q: Quaternion, tolerance: float = 1e-6) -> bool:
    """
    Check if a quaternion is normalized (unit length).

    Args:
        q: Quaternion to check
        tolerance: Tolerance for normalization check

    Returns:
        True if quaternion is normalized, False otherwise
    """
    magnitude = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    return abs(magnitude - 1.0) < tolerance