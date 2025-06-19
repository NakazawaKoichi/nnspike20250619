"""
Line Follower Control Module

This module contains implementations of line-following algorithms using camera-based
and color sensor-based methods. These implementations are used for training data
collection and real-time robot control. By sending input data such as a camera image
or sensor status, the functions return the necessary adjustments.

Functions:
    steer_by_camera(roi: np.ndarray) -> tuple[float, dict]:
        Calculates the steering adjustment based on the difference between the
        center of the Region of Interest (ROI) and the centroid of the largest contour
        in the x-coordinate.

    steer_by_reflection(reflection: int, threshold: int) -> int:
        Determines the steering adjustment based on light reflectivity detected by
        a color sensor, guiding the robot to follow the edge of a line.

    calculate_attitude_angle(offset_pixels: float, roi_bottom_y: int,
                           camera_height: float, focal_length_pixels: float) -> float:
        Calculates attitude angle (theta) from pixel offset using camera geometry
        for more accurate steering control.

    calculate_differential_steering(theta: float, base_speed: float, wheelbase: float) -> tuple[float, float]:
        Calculates individual wheel speeds for differential steering based on
        attitude angle and robot geometry.
"""

import cv2
import math
import numpy as np


def steer_by_camera(
    frame: np.ndarray, roi: tuple
) -> tuple[float, float, float, object]:
    """
    Processes a full camera frame to determine the steering direction based on contour detection within a specified ROI.

    Args:
        frame (np.ndarray): The full camera frame as a NumPy array.
        roi (tuple): Region of interest tuple (x1, y1, x2, y2).

    Returns:
        tuple[float, float, float, object]: A tuple containing:
            - mx (float): The x-coordinate of the centroid of the largest contour.
            - my (float): The y-coordinate of the centroid of the largest contour.
            - offset_pixels (float): Pixel offset from the center of the ROI.
            - max_contour (object): The largest contour detected in the ROI.

    The function performs the following steps:
        1. Extracts the region of interest from the full frame.
        2. Converts the ROI to grayscale.
        3. Applies Gaussian blur to the ROI to reduce noise.
        4. Converts the blurred image to a binary image using thresholding.
        5. Erodes and dilates the binary image to eliminate noise and restore eroded parts.
        6. Finds contours in the processed mask.
        7. Identifies the largest contour based on contour area.
        8. Calculates the moments of the largest contour to find its centroid.
        9. Calculates pixel offset from the center of the ROI.
    """
    # Extract ROI and convert to grayscale
    x1, y1, x2, y2 = roi
    roi_area = frame[y1:y2, x1:x2]
    image = cv2.cvtColor(roi_area, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(image, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

    # Erode to eliminate noise, Dilate to restore eroded parts of image
    mask = cv2.erode(thresh, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)

        mu = cv2.moments(max_contour)
        # Add 1e-5 to avoid division by zero
        mx = mu["m10"] / (mu["m00"] + 1e-5)
        my = mu["m01"] / (mu["m00"] + 1e-5)
    else:
        mx = image.shape[1] / 2
        my = image.shape[0] / 2
        max_contour = None

    # Calculate offset from center of ROI
    roi_center_x = image.shape[1] / 2
    offset_pixels = mx - roi_center_x

    return mx, my, offset_pixels, max_contour


def steer_by_reflection(reflection: int, threshold: int) -> int:
    """Determines the steering adjustment based on light reflectivity detected by a color sensor.
    The robot follows the edge of a line, turning one way when it detects more light reflectivity
    (whitish color) and the other way when it detects less light reflectivity (darkish color).

    Args:
        reflection (int): Light reflection value (white: 100, black: 0).
        threshold (int): The value to trigger the robot's turning.

    Returns:
        reflection_diff (int): The difference between the detected reflection value and the threshold.
    """
    reflection_diff = reflection - threshold
    return reflection_diff


def calculate_attitude_angle(
    offset_pixels: float,
    roi_bottom_y: int,
    camera_height: float = 0.20,
    focal_length_pixels: float = 640,
) -> float:
    """
    Calculate attitude angle (theta) from pixel offset using camera geometry.

    This function converts the pixel-based offset detected in the camera image
    to a real-world attitude angle that represents the robot's deviation from
    the desired path. This provides more physically meaningful control compared
    to simple pixel-based normalization.

    Args:
        offset_pixels (float): Lateral offset in pixels from image center
        roi_bottom_y (int): Bottom y-coordinate of ROI (closer to robot)
        camera_height (float, optional): Camera height above ground in meters. Defaults to 0.20.
        focal_length_pixels (float, optional): Camera focal length in pixels. Defaults to 640.

    Returns:
        float: Attitude angle (theta) in radians. Positive values indicate rightward deviation,
               negative values indicate leftward deviation.

    Note:
        The camera parameters (height and focal length) should be calibrated for your
        specific robot setup to ensure accurate angle calculations.
    """
    # Calculate ground distance from camera to the line detection point
    # Using similar triangles: ground_distance / camera_height = focal_length / (image_height - roi_bottom_y)
    image_height = 480  # Assuming standard camera resolution
    ground_distance = (
        camera_height * focal_length_pixels / (image_height - roi_bottom_y)
    )

    # Calculate lateral offset in meters
    # Using similar triangles: lateral_offset / ground_distance = offset_pixels / focal_length
    lateral_offset_meters = offset_pixels * ground_distance / focal_length_pixels

    # Calculate attitude angle (theta) using arctangent
    theta = math.atan2(lateral_offset_meters, ground_distance)

    return theta


def calculate_differential_steering(
    theta: float, base_speed: float, wheelbase: float = 0.15
) -> tuple[float, float]:
    """
    Calculate differential steering based on attitude angle and robot geometry.

    This function implements proper differential drive kinematics to calculate
    individual wheel speeds based on the desired turning angle. It uses the
    robot's wheelbase and the attitude angle to determine the appropriate
    speed difference between left and right wheels.

    Args:
        theta (float): Attitude angle in radians (positive = turn right, negative = turn left)
        base_speed (float): Base forward speed for both wheels when going straight
        wheelbase (float, optional): Distance between wheels in meters. Defaults to 0.15.

    Returns:
        tuple[float, float]: A tuple containing (left_speed, right_speed)
                           Individual wheel speeds for differential steering

    Note:
        - For straight line motion (theta ≈ 0), both wheels get the same speed
        - For turning, the inside wheel gets reduced speed, outside wheel gets increased speed
        - The wheelbase parameter should match your robot's actual wheel separation
    """
    # Handle straight line case to avoid division by zero
    if abs(theta) < 0.001:  # Threshold for "straight enough"
        return base_speed, base_speed

    # Calculate turn radius using differential drive kinematics
    # For differential drive: turn_radius = wheelbase / (2 * tan(theta))
    turn_radius = wheelbase / (2 * math.tan(abs(theta)))

    # Calculate speed ratio for differential steering
    # Inner wheel: v_inner = base_speed * (turn_radius - wheelbase/2) / turn_radius
    # Outer wheel: v_outer = base_speed * (turn_radius + wheelbase/2) / turn_radius
    speed_ratio_inner = (turn_radius - wheelbase / 2) / turn_radius
    speed_ratio_outer = (turn_radius + wheelbase / 2) / turn_radius

    if theta > 0:  # Turn right (positive theta)
        left_speed = base_speed * speed_ratio_outer  # Left wheel is outer
        right_speed = base_speed * speed_ratio_inner  # Right wheel is inner
    else:  # Turn left (negative theta)
        left_speed = base_speed * speed_ratio_inner  # Left wheel is inner
        right_speed = base_speed * speed_ratio_outer  # Right wheel is outer

    return left_speed, right_speed
