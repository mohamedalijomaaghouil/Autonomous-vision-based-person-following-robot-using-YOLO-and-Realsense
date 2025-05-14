import pyrealsense2 as rs
import numpy as np
import cv2
import mediapipe as mp
from pyModbusTCP.client import ModbusClient
import time
import ctypes
import struct

# Initialize MediaPipe Holistic
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

# Configurable parameters
FPS = 30
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
MAX_LIN = 0.6
MAX_ANGULAR = 0.6
TARGET_DISTANCE = 2
MIN_STANDBY_DISTANCE = 1.90
MAX_STANDBY_DISTANCE = 2.10
KP_LINEAR = 0.7
KP_ANGULAR = 1.2
SCALE = 100
LINEAR_SPEED_REGISTER = 100
ANGULAR_SPEED_REGISTER = 101
ANALOG_DEAD_ZONE = 0.01
GESTURE_COOLDOWN_FRAMES = 60  # 3s
GESTURE_CONFIRM_FRAMES = 1
NO_TARGET_TIMEOUT_FRAMES = 200  # 10s
LOOP_RATE = 0.05  # 20 Hz
SMOOTHING_FACTOR = 0.7  # For tracking
SHOULDER_LOSS_HOLD_FRAMES = 200  # 10s at 20 Hz
HAND_VISIBILITY_THRESHOLD = 0.1  # Lowered for better detection at distance
FALLBACK_VISIBILITY_THRESHOLD = 0.1  # Lowered for better detection at distance
THUMB_INDEX_DISTANCE_THRESHOLD = 0.07  # Increased for better detection at distance
ALIGNMENT_MARGIN = 50  # Pixels for slope difference

# Initialize Modbus client
modbus_client = ModbusClient(host="127.0.0.1", port=1502)

# Attempt to open Modbus connection
def open_modbus_connection():
    if not modbus_client.is_open:
        try:
            if modbus_client.open():
                print("Modbus connection opened successfully.")
                return True
            else:
                print("Failed to open Modbus connection. Running in Simulation Mode.")
                return False
        except Exception as e:
            print(f"Error opening Modbus connection: {e}")
            return False
    return True

# Function to average depth over a small region
def get_average_depth(depth_image, x, y, size=5):
    x, y = int(x), int(y)
    half_size = size // 2
    x_min = max(0, x - half_size)
    x_max = min(depth_image.shape[1], x + half_size + 1)
    y_min = max(0, y - half_size)
    y_max = min(depth_image.shape[0], y + half_size + 1)
    region = depth_image[y_min:y_max, x_min:x_max]
    valid_depths = region[region > 0]
    return np.mean(valid_depths) if valid_depths.size > 0 else 0

# Initialize RealSense pipeline
print("Setting up RealSense pipeline...")
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)

# Align depth to color stream
align = rs.align(rs.stream.color)

# Check for RealSense device
context = rs.context()
if not context.devices:
    print("Error: No RealSense device connected!")
    open_modbus_connection()
    try:
        modbus_client.write_multiple_registers(LINEAR_SPEED_REGISTER, [0, 0])
        print("Sent initial stop command to Modbus.")
    except Exception as e:
        print(f"Failed to send initial stop command: {e}")
    exit(1)
else:
    print(f"Found {len(context.devices)} RealSense device(s):")
    for device in context.devices:
        print(f" - {device.get_info(rs.camera_info.name)} (Serial: {device.get_info(rs.camera_info.serial_number)})")

# Attempt initial Modbus connection and test write
if open_modbus_connection():
    try:
        modbus_client.write_multiple_registers(LINEAR_SPEED_REGISTER, [0, 0])
        print("Successfully sent test stop command to Modbus.")
    except Exception as e:
        print(f"Failed to send test stop command: {e}")

# Start pipeline
try:
    print("Starting pipeline...")
    profile = pipe.start(cfg)
    print("Pipeline started!")

    # Get depth scale for visualization
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Initialize Holistic model with adjusted confidence for better detection at distance
    with mp_holistic.Holistic(
        min_detection_confidence=0.5,  # Lowered for better detection at distance
        min_tracking_confidence=0.5,  # Lowered for better detection at distance
        enable_segmentation=False,
        refine_face_landmarks=False,
        model_complexity=1
    ) as holistic:
        # Initialize variables
        Vx = 0.0
        Wz = 0.0
        pipeline_started = True
        target_body_id = None
        is_following = False
        last_following_state = False
        gesture_mode = False
        gesture_cooldown = 0
        gesture_confirm_counter = 0
        no_target_frames = 0
        shoulder_loss_frames = 0
        chest_x_smooth = None
        chest_y_smooth = None
        chest_z_smooth = None
        Vx_smooth = 0.0
        Wz_smooth = 0.0
        last_valid_chest_x = None
        last_valid_chest_y = None
        last_valid_chest_z = None

        while True:
            try:
                start_time = time.time()

                # Wait for frames
                frames = pipe.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    print("No frame received, skipping...")
                    Vx = 0.0
                    Wz = 0.0
                    no_target_frames += 1
                    continue

                # Convert frames to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # Recolor for MediaPipe (BGR to RGB)
                image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                # Make detections
                results = holistic.process(image_rgb)

                # Recolor back to BGR for rendering
                image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

                # Draw pose landmarks
                if results.pose_landmarks:
                    mp_drawing.draw_landmarks(
                        image_bgr,
                        results.pose_landmarks,
                        mp_holistic.POSE_CONNECTIONS,
                        mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=4),
                        mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                    )

                    # Draw right-hand landmarks with distinct color if detected
                    if results.right_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image_bgr,
                            results.right_hand_landmarks,
                            mp_holistic.HAND_CONNECTIONS,
                            mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),  # Red for hand
                            mp_drawing.DrawingSpec(color=(0, 255, 255), thickness=2, circle_radius=2)
                        )

                    # Lock first detected person as target
                    if target_body_id is None:
                        target_body_id = 1
                        is_following = False
                        last_following_state = False
                        print(f"Locked Target (Body ID {target_body_id}), Restored mode: {'Following' if is_following else 'Not Following'}")

                    # Track only the locked target
                    if target_body_id is not None:
                        # Calculate chest center using midpoint of shoulders
                        shoulder_left = results.pose_landmarks.landmark[11]
                        shoulder_right = results.pose_landmarks.landmark[12]
                        if shoulder_left.visibility > 0.3 and shoulder_right.visibility > 0.3:
                            x_left = int(shoulder_left.x * COLOR_WIDTH)
                            x_right = int(shoulder_right.x * COLOR_WIDTH)
                            y_left = int(shoulder_left.y * COLOR_HEIGHT)
                            y_right = int(shoulder_right.y * COLOR_HEIGHT)
                            if (0 <= x_left < DEPTH_WIDTH and 0 <= y_left < DEPTH_HEIGHT and
                                0 <= x_right < DEPTH_WIDTH and 0 <= y_right < DEPTH_HEIGHT):
                                chest_center_x = (x_left + x_right) / 2
                                chest_center_y = (y_left + y_right) / 2
                                chest_center_z = get_average_depth(depth_image, chest_center_x, chest_center_y) * depth_scale
                                if 0.5 < chest_center_z < 5.0:
                                    # Smooth tracking coordinates
                                    if chest_x_smooth is None:
                                        chest_x_smooth = chest_center_x
                                        chest_y_smooth = chest_center_y
                                        chest_z_smooth = chest_center_z
                                    else:
                                        chest_x_smooth = SMOOTHING_FACTOR * chest_center_x + (1 - SMOOTHING_FACTOR) * chest_x_smooth
                                        chest_y_smooth = SMOOTHING_FACTOR * chest_center_y + (1 - SMOOTHING_FACTOR) * chest_y_smooth
                                        chest_z_smooth = SMOOTHING_FACTOR * chest_center_z + (1 - SMOOTHING_FACTOR) * chest_z_smooth

                                    # Update last valid coordinates
                                    last_valid_chest_x = chest_x_smooth
                                    last_valid_chest_y = chest_y_smooth
                                    last_valid_chest_z = chest_z_smooth

                                    # Draw marker at smoothed center
                                    cv2.circle(image_bgr, (int(chest_x_smooth), int(chest_y_smooth)), 5, (0, 0, 255), -1)
                                    cv2.putText(image_bgr, f"Distance: {chest_z_smooth:.2f} m", (int(chest_x_smooth), int(chest_y_smooth - 10)),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                                    # Reset counters
                                    no_target_frames = 0
                                    shoulder_loss_frames = 0

                                    # Single-hand stop gesture detection
                                    if gesture_mode and gesture_cooldown <= 0:
                                        right_wrist = results.pose_landmarks.landmark[16]
                                        right_shoulder = results.pose_landmarks.landmark[12]
                                        right_elbow = results.pose_landmarks.landmark[14]
                                        x_right_wrist = int(right_wrist.x * COLOR_WIDTH)
                                        y_right_wrist = int(right_wrist.y * COLOR_HEIGHT)
                                        wrist_depth = get_average_depth(depth_image, x_right_wrist, y_right_wrist) * depth_scale

                                        # Primary gesture: Open hand with aligned arm
                                        gesture_detected = False
                                        print(f"Hand landmarks detected: {bool(results.right_hand_landmarks)}")
                                        if (right_wrist.visibility > HAND_VISIBILITY_THRESHOLD and
                                            right_shoulder.visibility > HAND_VISIBILITY_THRESHOLD and
                                            right_elbow.visibility > HAND_VISIBILITY_THRESHOLD and
                                            results.right_hand_landmarks and
                                            0.5 <= wrist_depth <= 5.0):  # Extended depth range
                                            thumb_tip = results.right_hand_landmarks.landmark[4]
                                            index_tip = results.right_hand_landmarks.landmark[8]
                                            middle_tip = results.right_hand_landmarks.landmark[12]

                                            if (thumb_tip.visibility > HAND_VISIBILITY_THRESHOLD and
                                                index_tip.visibility > HAND_VISIBILITY_THRESHOLD and
                                                middle_tip.visibility > HAND_VISIBILITY_THRESHOLD):
                                                thumb_index_distance = np.sqrt(
                                                    (thumb_tip.x - index_tip.x)**2 +
                                                    (thumb_tip.y - index_tip.y)**2
                                                )
                                                # Calculate alignment in pixels
                                                elbow_x = int(right_elbow.x * COLOR_WIDTH)
                                                elbow_y = int(right_elbow.y * COLOR_HEIGHT)
                                                shoulder_x = int(right_shoulder.x * COLOR_WIDTH)
                                                shoulder_y = int(right_shoulder.y * COLOR_HEIGHT)
                                                wrist_x = x_right_wrist
                                                wrist_y = y_right_wrist
                                                slope_diff = abs((shoulder_y - elbow_y) / (shoulder_x - elbow_x + 1e-6) -
                                                                 (wrist_y - shoulder_y) / (wrist_x - shoulder_x + 1e-6))
                                                slope_diff_pixels = abs((shoulder_y - elbow_y) - (wrist_y - shoulder_y))

                                                print(f"Wrist y={right_wrist.y:.2f}, Shoulder y={right_shoulder.y:.2f}, "
                                                      f"Thumb-index distance={thumb_index_distance:.2f}, "
                                                      f"Visibility: Wrist={right_wrist.visibility:.2f}, "
                                                      f"Thumb={thumb_tip.visibility:.2f}, Index={index_tip.visibility:.2f}, "
                                                      f"Middle={middle_tip.visibility:.2f}, "
                                                      f"Slope diff pixels={slope_diff_pixels:.2f}, Depth={wrist_depth:.2f}, "
                                                      f"Coords: ({x_right_wrist}, {y_right_wrist})")

                                                if (0 <= x_right_wrist < DEPTH_WIDTH and 0 <= y_right_wrist < DEPTH_HEIGHT and
                                                    right_wrist.y < right_shoulder.y and
                                                    thumb_index_distance > THUMB_INDEX_DISTANCE_THRESHOLD and
                                                    slope_diff_pixels < ALIGNMENT_MARGIN):
                                                    gesture_confirm_counter += 1
                                                    print(f"Primary gesture detected, confirming: {gesture_confirm_counter}/{GESTURE_CONFIRM_FRAMES}")
                                                    radius = 10 + (gesture_confirm_counter * 2)
                                                    cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), radius, (0, 255, 0), 2)
                                                    cv2.line(image_bgr, (elbow_x, elbow_y), (shoulder_x, shoulder_y), (0, 255, 0), 2)
                                                    cv2.line(image_bgr, (shoulder_x, shoulder_y), (wrist_x, wrist_y), (0, 255, 0), 2)

                                                    if gesture_confirm_counter >= GESTURE_CONFIRM_FRAMES:
                                                        gesture_detected = True
                                                        print(f"Primary gesture confirmed: {'Following' if not is_following else 'Not Following'} activated")
                                                else:
                                                    gesture_confirm_counter = 0
                                                    print("Primary gesture conditions not met: Insufficient y-distance, thumb-index distance, or alignment")
                                                    cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), 10, (0, 255, 255), 2)
                                            else:
                                                gesture_confirm_counter = 0
                                                print("Hand landmark visibility too low")
                                                cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), 10, (0, 255, 255), 2)
                                        else:
                                            gesture_confirm_counter = 0
                                            print("Primary gesture failed: Low visibility or out of depth range")
                                            cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), 10, (0, 255, 255), 2)

                                        # Fallback: Wrist-only detection
                                        if not gesture_detected and right_wrist.visibility > FALLBACK_VISIBILITY_THRESHOLD and right_shoulder.visibility > HAND_VISIBILITY_THRESHOLD:
                                            print(f"Fallback check: Wrist y={right_wrist.y:.2f}, Shoulder y={right_shoulder.y:.2f}, "
                                                  f"Visibility: Wrist={right_wrist.visibility:.2f}, Shoulder={right_shoulder.visibility:.2f}, "
                                                  f"Depth={wrist_depth:.2f}, Coords: ({x_right_wrist}, {y_right_wrist})")
                                            if (0 <= x_right_wrist < DEPTH_WIDTH and 0 <= y_right_wrist < DEPTH_HEIGHT and
                                                right_wrist.y < right_shoulder.y and
                                                0.5 <= wrist_depth <= 5.0):  # Extended depth range
                                                gesture_confirm_counter += 1
                                                print(f"Fallback gesture detected, confirming: {gesture_confirm_counter}/{GESTURE_CONFIRM_FRAMES}")
                                                radius = 10 + (gesture_confirm_counter * 2)
                                                cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), radius, (0, 255, 128), 2)
                                                if gesture_confirm_counter >= GESTURE_CONFIRM_FRAMES:
                                                    gesture_detected = True
                                                    print(f"Fallback gesture confirmed: {'Following' if not is_following else 'Not Following'} activated")
                                            else:
                                                gesture_confirm_counter = 0
                                                print("Fallback gesture conditions not met: Insufficient y-distance or out of depth range")
                                                cv2.circle(image_bgr, (x_right_wrist, y_right_wrist), 10, (0, 255, 255), 2)

                                        if gesture_detected:
                                            is_following = not is_following
                                            gesture_cooldown = GESTURE_COOLDOWN_FRAMES
                                            gesture_confirm_counter = 0

                                    if gesture_cooldown > 0:
                                        gesture_cooldown -= 1

                                    # Calculate velocities
                                    Vx = 0.0
                                    Wz = 0.0
                                    if is_following:
                                        image_center_x = COLOR_WIDTH / 2
                                        if MIN_STANDBY_DISTANCE <= chest_z_smooth <= MAX_STANDBY_DISTANCE:
                                            Vx = 0.0
                                            Wz = 0.0
                                            Vx_smooth = 0.0
                                            Wz_smooth = 0.0
                                            print(f"In standby range: Distance={chest_z_smooth:.2f} m, Vx_smooth={Vx_smooth:.2f}, Wz_smooth={Wz_smooth:.2f}")
                                        else:
                                            distance_error = chest_z_smooth - TARGET_DISTANCE
                                            Vx = KP_LINEAR * distance_error
                                            Vx = max(0.0, Vx)
                                            Vx = np.clip(Vx, 0.0, MAX_LIN)
                                            angular_error = (image_center_x - chest_x_smooth) / image_center_x
                                            Wz = KP_ANGULAR * angular_error
                                            Wz = np.clip(Wz, -MAX_ANGULAR, MAX_ANGULAR)
                                            print(f"Following: Distance={chest_z_smooth:.2f} m, Vx={Vx:.2f}, Wz={Wz:.2f}")

                                    # Smooth velocities
                                    Vx_smooth = SMOOTHING_FACTOR * Vx + (1 - SMOOTHING_FACTOR) * Vx_smooth
                                    Wz_smooth = SMOOTHING_FACTOR * Wz + (1 - SMOOTHING_FACTOR) * Wz_smooth
                                    Vx = Vx_smooth
                                    Wz = Wz_smooth

                                else:
                                    print("Target out of depth range, holding last mode...")
                                    shoulder_loss_frames += 1
                            else:
                                print("Target out of frame, holding last mode...")
                                shoulder_loss_frames += 1
                        else:
                            print("Shoulders not detected, holding last mode...")
                            shoulder_loss_frames += 1

                        # Handle shoulder loss hold period
                        if shoulder_loss_frames > 0 and shoulder_loss_frames <= SHOULDER_LOSS_HOLD_FRAMES:
                            if last_valid_chest_x is not None:
                                chest_x_smooth = last_valid_chest_x
                                chest_y_smooth = last_valid_chest_y
                                chest_z_smooth = last_valid_chest_z
                                cv2.circle(image_bgr, (int(chest_x_smooth), int(chest_y_smooth)), 5, (0, 255, 255), -1)
                                cv2.putText(image_bgr, f"Distance: {chest_z_smooth:.2f} m (Hold)", (int(chest_x_smooth), int(chest_y_smooth - 10)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                                print(f"Holding last mode: {'Following' if is_following else 'Not Following'}, Frames: {shoulder_loss_frames}/{SHOULDER_LOSS_HOLD_FRAMES}")
                                no_target_frames = 0
                            else:
                                Vx = 0.0
                                Wz = 0.0
                                no_target_frames += 1
                        elif shoulder_loss_frames > SHOULDER_LOSS_HOLD_FRAMES:
                            print("Shoulder loss timeout, stopping robot...")
                            Vx = 0.0
                            Wz = 0.0
                            no_target_frames += 1
                            shoulder_loss_frames = 0
                            last_valid_chest_x = None
                            last_valid_chest_y = None
                            last_valid_chest_z = None

                    else:
                        print("No target detected, stopping robot...")
                        Vx = 0.0
                        Wz = 0.0
                        no_target_frames += 1

                else:
                    print("No pose landmarks detected, stopping robot...")
                    Vx = 0.0
                    Wz = 0.0
                    no_target_frames += 1
                    if no_target_frames >= NO_TARGET_TIMEOUT_FRAMES:
                        print("Target lost for too long, resetting tracking...")
                        last_following_state = is_following
                        is_following = False
                        target_body_id = None
                        no_target_frames = 0
                        shoulder_loss_frames = 0
                        last_valid_chest_x = None
                        last_valid_chest_y = None
                        last_valid_chest_z = None
                        cv2.putText(image_bgr, "Target Lost - Robot Stopped", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Display status on color feed
                mode_text = "Real Mode" if modbus_client.is_open else "Simulation Mode"
                cv2.putText(image_bgr, mode_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                gesture_mode_text = "Gesture Mode: ON (Press 'u' to disable)" if gesture_mode else "Gesture Mode: OFF (Press 'c' to enable)"
                gesture_mode_color = (0, 255, 0) if gesture_mode else (255, 255, 255)
                cv2.putText(image_bgr, gesture_mode_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, gesture_mode_color, 2)
                status_text = "Following" if is_following else "Not Following"
                cv2.putText(image_bgr, status_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0) if is_following else (0, 0, 255), 2)

                # Display color feed
                cv2.imshow('RealSense Color Feed with Landmarks', image_bgr)

                # Handle keyboard input
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    print("Quitting...")
                    break
                elif key == ord('c'):
                    if not gesture_mode:
                        gesture_mode = True
                        print("Gesture mode activated (press 'u' to deactivate)")
                elif key == ord('u'):
                    if gesture_mode:
                        gesture_mode = False
                        print("Gesture mode deactivated (press 'c' to activate)")
                elif key == ord('f'):
                    is_following = not is_following
                    print(f"Manual toggle: {'Following' if is_following else 'Not Following'} activated")

                # Send velocities to Modbus server or simulate
                try:
                    is_moving = abs(Vx) > ANALOG_DEAD_ZONE or abs(Wz) > ANALOG_DEAD_ZONE
                    if not modbus_client.is_open:
                        open_modbus_connection()
                    if modbus_client.is_open:
                        Vx_scaled = int(Vx * SCALE)
                        Wz_scaled = int(Wz * SCALE)
                        Vx_16bit = ctypes.c_int16(Vx_scaled).value
                        Wz_16bit = ctypes.c_int16(Wz_scaled).value
                        packed_Vx = struct.pack('>h', Vx_16bit)
                        packed_Wz = struct.pack('>h', Wz_16bit)
                        Vx_uint16 = struct.unpack('>H', packed_Vx)[0]
                        Wz_uint16 = struct.unpack('>H', packed_Wz)[0]
                        print(f"Sending to Modbus: Vx_scaled={Vx_scaled}, Wz_scaled={Wz_scaled}, Registers=[{Vx_uint16}, {Wz_uint16}]")
                        modbus_client.write_multiple_registers(LINEAR_SPEED_REGISTER, [Vx_uint16, Wz_uint16])
                        print(f"Real Mode | Vx: {Vx:.2f} m/s, Wz: {Wz:.2f} rad/s, Moving: {is_moving}")
                    else:
                        print(f"Simulation Mode | Vx: {Vx:.2f} m/s, Wz: {Wz:.2f} rad/s, Moving: {is_moving}")
                except Exception as e:
                    print(f"Error during Modbus communication: {e}")
                    print(f"Simulation Mode | Vx: {Vx:.2f} m/s, Wz: {Wz:.2f} rad/s, Moving: {is_moving}")

                # Control loop rate
                end_time = time.time()
                elapsed = end_time - start_time
                print(f"Frame processing time: {elapsed:.3f} s")
                sleep_time = max(0, LOOP_RATE - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                print(f"Error processing frame: {e}")
                continue

except Exception as e:
    print(f"Pipeline error: {e}")
    try:
        device = context.devices[0]
        sensor = device.first_depth_sensor()
        print("Supported depth stream configurations:")
        for stream in sensor.get_stream_profiles():
            if stream.stream_type() == rs.stream.depth:
                print(f" - {stream}")
        sensor = device.query_sensors()[1]
        print("Supported color stream configurations:")
        for stream in sensor.get_stream_profiles():
            if stream.stream_type() == rs.stream.color:
                print(f" - {stream}")
    except Exception as e:
        print(f"Error querying stream configurations: {e}")
finally:
    print("Stopping pipeline...")
    if modbus_client.is_open:
        try:
            modbus_client.write_multiple_registers(LINEAR_SPEED_REGISTER, [0, 0])
            print("Sent final stop command to Modbus.")
        except Exception as e:
            print(f"Error sending stop command: {e}")
        modbus_client.close()
    if pipeline_started:
        pipe.stop()
    cv2.destroyAllWindows()
