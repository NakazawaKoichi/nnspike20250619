#!/usr/bin/env python3
"""
OpenCV-Based Line Following Robot Control

This script controls a line-following robot using OpenCV computer vision
instead of neural network predictions. It uses the steer_by_camera function
to detect the line centroid and follows it using PID control.

Speed Tuning Parameters:
- BASE_POWER: Base power for straight lines (start here)
- TURN_REDUCTION_FACTOR: Speed reduction in turns (0.0-1.0)
- TURN_THRESHOLD: Pixel offset threshold to detect turns

PID Tuning Parameters:
- Kp, Ki, Kd: Standard PID parameters for steering correction
"""
import cv2
import math
import time
import socket
import pickle
import struct
import argparse
import numpy as np
from nnspike.unit import ETRobot
from nnspike.utils import (
    steer_by_camera,
    draw_driving_info,
    PIDController,
    SensorRecorder,
    calculate_attitude_angle,
    calculate_differential_steering,
)
from nnspike.constants import (
    ROI_OPENCV,
    CAMERA_HEIGHT,
    CAMERA_FOCAL_LENGTH_PIXELS,
    WHEELBASE,
)

# User defined constants
x1, y1, x2, y2 = ROI_OPENCV  # Region of Interest for OpenCV processing

# Simplified Speed Control Parameters (Easy to tune)
BASE_POWER = 10  # Base power for straight lines (adjust this first)
TURN_REDUCTION_FACTOR = 0.6  # Reduce speed in turns (0.0-1.0, lower = slower in turns)
TURN_THRESHOLD = 50  # Pixel offset threshold to detect turns (pixels from center)

# Socket connection settings
HOST_IP_ADDRESS = (
    "192.168.137.1"  # The destination IP(PC) that the Raspberry Pi will send to
)

# Camera setup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def main(record_sensor_data=False, save_camera_video=False):
    # Generate timestamp for consistent naming if recording is enabled
    TIMESTAMP = (
        time.strftime("%Y%m%d%H%M%S", time.localtime())
        if (record_sensor_data or save_camera_video)
        else None
    )

    # Initialize sensor recorder conditionally
    sensor_recorder = None
    if record_sensor_data:
        sensor_recorder = SensorRecorder(timestamp=TIMESTAMP)
        sensor_recorder.start_recording()

    # Initialize video writer conditionally
    video_writer = None
    if save_camera_video:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        video_filename = f"storage/videos/{TIMESTAMP}_picamera.avi"
        video_writer = cv2.VideoWriter(
            filename=video_filename,
            fourcc=fourcc,
            fps=30,
            frameSize=(640, 480),
        )

    # Socket connection for sending camera capture
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(
        (HOST_IP_ADDRESS, 8485)
    )  # Initialize robot and PID controller
    print("シリアルポートに接続中...")
    et = ETRobot()
    print("接続成功")    # アームの初期化
    print("アームを初期化します...")
    print("アームを上に動かします...")
    print("コマンド送信: COMMAND_MOVE_ARM_ID=205, action=1")
    et.move_arm(action=1)
    time.sleep(1)  # 動作完了を待機
    
    print("アームを下に動かします...")
    print("コマンド送信: COMMAND_MOVE_ARM_ID=205, action=0")
    et.move_arm(action=0)
    time.sleep(1)  # 動作完了を待機

    # モーター状態を確認
    status = et.get_spike_status()
    print(f"モーターC（アーム）の状態: {status.motors['C']}")

    pid = PIDController(
        Kp=15,  # Reduced for OpenCV (smoother than neural network)
        Ki=0,
        Kd=3,
        setpoint=0,
        output_limits=(-0.25, 0.25),  # Direct radian limits for steering correction
    )

    time.sleep(0.5)
    et.set_motor_relative_position(left_positon=0, right_position=0)

    try:
        while et.is_running == True:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break  # Save video frame if enabled
            if save_camera_video and video_writer is not None:
                video_writer.write(frame)

            # Process frame for steering using updated steer_by_camera function
            mx, my, offset_pixels, max_contour = steer_by_camera(frame, ROI_OPENCV)

            # Calculate attitude angle using camera geometry
            theta = calculate_attitude_angle(
                offset_pixels, y2, CAMERA_HEIGHT, CAMERA_FOCAL_LENGTH_PIXELS
            )

            # Simple speed control: reduce speed in turns for easier tuning
            abs_offset = abs(offset_pixels)
            if abs_offset > TURN_THRESHOLD:  # In a turn
                current_base_power = BASE_POWER * TURN_REDUCTION_FACTOR
            else:  # Going straight
                current_base_power = (
                    BASE_POWER  # Use PID controller with attitude angle
                )
            steering_correction = pid.update(theta)

            # Apply steering with geometric approach
            left_power, right_power = calculate_differential_steering(
                theta + steering_correction, current_base_power, WHEELBASE
            )

            # Clamp power values to valid range
            left_power = int(max(0, min(100, left_power)))
            right_power = int(max(0, min(100, right_power)))

            et.set_motor_forward_power(
                left_power=left_power,
                right_power=right_power,
            )

            # Log sensor data using the recorder if enabled
            if record_sensor_data and sensor_recorder is not None:
                sensor_recorder.log_frame_data(et.get_spike_status())

            # Prepare driving information for visualization
            info = dict()
            info["offset_x"], info["offset_y"] = x1 + mx, y1 + my
            info["text"] = {
                "theta_deg": round(math.degrees(theta), 2),
                "steering_correction": round(steering_correction, 2),
                "offset_pixels": round(offset_pixels, 1),
                "base_power": int(current_base_power),
                "left_power": left_power,
                "right_power": right_power,
                "contour_area": (
                    int(cv2.contourArea(max_contour)) if max_contour is not None else 0
                ),
            }

            # Create visualization frame
            gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
            gray = draw_driving_info(gray, info, (x1, y1, x2, y2))

            # Draw contour on the visualization if found
            if max_contour is not None:
                # Adjust contour coordinates to full frame
                adjusted_contour = max_contour + np.array([x1, y1])
                cv2.drawContours(gray, [adjusted_contour], -1, (255, 255, 255), 2)

                # Draw centroid
                cv2.circle(gray, (int(x1 + mx), int(y1 + my)), 5, (255, 255, 255), -1)

            # Send camera capture for remote monitoring
            try:
                ret, buffer = cv2.imencode(".png", gray)
                img_encoded = buffer.tobytes()
                data = pickle.dumps(img_encoded)
                client_socket.sendall(struct.pack("L", len(data)) + data)
            except Exception as e:
                print(f"Socket error: {e}")
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        et.stop()
        cap.release()
        client_socket.close()

        # Clean up video writer if it was used
        if save_camera_video and video_writer is not None:
            video_writer.release()
            print(f"Video saved to: {video_filename}")

        # Clean up sensor recorder if it was used
        if record_sensor_data and sensor_recorder is not None:
            sensor_recorder.stop_recording()
            print(f"Total frames recorded: {sensor_recorder.get_frame_count()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run the OpenCV-based line following robot with optional sensor recording and video saving"
    )
    parser.add_argument(
        "--record-sensor", action="store_true", help="Record sensor data to file"
    )
    parser.add_argument(
        "--save-video", action="store_true", help="Save camera video to file"
    )

    args = parser.parse_args()

    print("Starting OpenCV-based line following robot...")
    print(f"Using ROI: {ROI_OPENCV}")
    print(f"Base power: {BASE_POWER}")
    print(f"Turn reduction factor: {TURN_REDUCTION_FACTOR}")
    print(f"Turn threshold: {TURN_THRESHOLD} pixels")
    print("Press Ctrl+C to stop")

    main(record_sensor_data=args.record_sensor, save_camera_video=args.save_video)
