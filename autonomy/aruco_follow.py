from pymavlink import mavutil
import cv2
import numpy as np
import time
import pyrealsense2 as rs

# MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()

# Set mode with verification
print("Setting GUIDED mode...")
master.set_mode('GUIDED')
time.sleep(2)

# Ground vehicle parameters (forward-only)
CRUISE_SPEED = 0.3
TARGET_DISTANCE_M = 1.0
DEADBAND_M = 0.1
FRAME_RATE = 30
MARKER_SIZE_M = 0.254

# Modern OpenCV ArUco setup
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

class ForwardOnlyPID:
    def __init__(self):
        # Forward motion control (no reverse)
        self.forward_kp = 0.8
        self.forward_ki = 0.02
        self.forward_kd = 0.1
        
        # Turning control
        self.turn_kp = 1.2  # Increased for more aggressive turning
        self.turn_ki = 0.03
        self.turn_kd = 0.15
        
        # Forward control state
        self.forward_integral = 0
        self.forward_last_error = 0
        self.forward_last_time = time.time()
        
        # Turning control state
        self.turn_integral = 0
        self.turn_last_error = 0
        self.turn_last_time = time.time()
        
        self.max_integral = 0.2

    def compute_forward(self, distance_error):
        """Only compute forward velocity (never negative)"""
        current_time = time.time()
        dt = current_time - self.forward_last_time
        
        if dt > 0.01:
            # Only allow forward motion when marker is far enough
            if distance_error > 0:  # Too far - move forward
                proportional = self.forward_kp * distance_error
                
                self.forward_integral += distance_error * dt
                self.forward_integral = np.clip(self.forward_integral, 0, self.max_integral)
                integral_term = self.forward_ki * self.forward_integral
                
                derivative = (distance_error - self.forward_last_error) / dt
                derivative_term = self.forward_kd * derivative
                
                output = proportional + integral_term + derivative_term
                output = np.clip(output, 0, CRUISE_SPEED)  # Only positive velocities
                
                self.forward_last_error = distance_error
                self.forward_last_time = current_time
                
                return output
            else:
                # Too close - stop (no reverse)
                self.forward_integral = 0
                return 0
        return 0

    def compute_turn(self, angle_error):
        """Compute turning velocity"""
        current_time = time.time()
        dt = current_time - self.turn_last_time
        
        if dt > 0.01:
            proportional = self.turn_kp * angle_error
            
            self.turn_integral += angle_error * dt
            self.turn_integral = np.clip(self.turn_integral, -self.max_integral, self.max_integral)
            integral_term = self.turn_ki * self.turn_integral
            
            derivative = (angle_error - self.turn_last_error) / dt
            derivative_term = self.turn_kd * derivative
            
            output = proportional + integral_term + derivative_term
            output = np.clip(output, -1.5, 1.5)  # Increased turning rate
            
            self.turn_last_error = angle_error
            self.turn_last_time = current_time
            
            return output
        return 0

    def reset(self):
        self.forward_integral = 0
        self.forward_last_error = 0
        self.forward_last_time = time.time()
        
        self.turn_integral = 0
        self.turn_last_error = 0
        self.turn_last_time = time.time()

def setup_realsense():
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
        
        profile = pipeline.start(config)
        
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                 [0, intrinsics.fy, intrinsics.ppy],
                                 [0, 0, 1]])
        
        dist_coeffs = np.array(intrinsics.coeffs)
        
        for _ in range(60):
            pipeline.wait_for_frames()
        
        align = rs.align(rs.stream.color)
        print("RealSense initialized for forward-only vehicle")
        return pipeline, align, camera_matrix, dist_coeffs
        
    except Exception as e:
        print(f"Camera initialization failed: {str(e)}")
        raise

def send_velocity_ground(vx, yaw_rate):
    """Send velocity command - vx is always >= 0"""
    try:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0, vx, 0, 0, 0, 0, 0, 0, yaw_rate
        )
        return True
    except Exception as e:
        print(f"Velocity command failed: {e}")
        return False

def detect_aruco_marker(color_frame, depth_frame, camera_matrix, dist_coeffs):
    color_img = np.asanyarray(color_frame.get_data())
    depth_img = np.asanyarray(depth_frame.get_data())
    
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    
    corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(color_img, corners, ids)
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE_M, camera_matrix, dist_coeffs
        )
        
        marker_id = ids[0][0]
        rvec = rvecs[0]
        tvec = tvecs[0]
        
        cv2.drawFrameAxes(color_img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
        
        distance = np.linalg.norm(tvec)
        angle_rad = np.arctan2(tvec[0][0], tvec[0][2])
        angle_deg = np.degrees(angle_rad)
        
        marker_center = np.mean(corners[0][0], axis=0).astype(int)
        
        cv2.putText(color_img, f"ID: {marker_id}", (marker_center[0]-30, marker_center[1]-40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(color_img, f"Dist: {distance:.2f}m", (marker_center[0]-30, marker_center[1]-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(color_img, f"Angle: {angle_deg:.1f}°", (marker_center[0]-30, marker_center[1]),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return distance, angle_rad, color_img, marker_center
    
    return None, None, color_img, None

def forward_only_search(search_state):
    """Search pattern for forward-only vehicle"""
    patterns = {
        'turn_left': (0, 0.5),      # Turn left in place
        'forward_left': (0.15, 0.3), # Move forward while turning left
        'turn_right': (0, -0.5),    # Turn right in place
        'forward_right': (0.15, -0.3), # Move forward while turning right
        'forward': (0.2, 0),        # Move straight forward
    }
    
    if search_state['pattern'] in patterns:
        vx, yaw_rate = patterns[search_state['pattern']]
        send_velocity_ground(vx, yaw_rate)
        
        search_state['counter'] += 1
        
        # Change pattern every 45 frames (1.5 seconds at 30fps)
        if search_state['counter'] >= 45:
            search_state['counter'] = 0
            current_patterns = list(patterns.keys())
            current_idx = current_patterns.index(search_state['pattern'])
            search_state['pattern'] = current_patterns[(current_idx + 1) % len(current_patterns)]
    
    return search_state

# Main execution
try:
    pipeline, align, camera_matrix, dist_coeffs = setup_realsense()
    pid = ForwardOnlyPID()
    
    search_state = {'pattern': 'turn_left', 'counter': 0}
    target_lost_count = 0
    overshoot_count = 0
    
    print("Starting forward-only ArUco marker approach...")
    
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            aligned = align.process(frames)
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            
            if not color or not depth:
                continue
                
            distance, angle, frame, marker_center = detect_aruco_marker(
                color, depth, camera_matrix, dist_coeffs
            )
            
            if distance is not None and angle is not None:
                target_lost_count = 0
                
                distance_error = distance - TARGET_DISTANCE_M
                angle_error = angle
                
                # Check if we're at target
                if abs(distance_error) < DEADBAND_M and abs(angle_error) < np.radians(5):
                    send_velocity_ground(0, 0)
                    status = f"TARGET REACHED! D:{distance:.2f}m A:{np.degrees(angle):.1f}°"
                    color_status = (0, 255, 0)
                    overshoot_count = 0
                
                # Check if we overshot (too close and can't reverse)
                elif distance_error < -0.2:  # More than 20cm too close
                    overshoot_count += 1
                    if overshoot_count > 30:  # 1 second of being too close
                        # Execute turning maneuver to reposition
                        if abs(angle_error) > np.radians(10):
                            # Turn to face marker better
                            yaw_rate = 0.8 if angle_error > 0 else -0.8
                            send_velocity_ground(0, yaw_rate)
                            status = f"REPOSITIONING - TURN {'RIGHT' if angle_error > 0 else 'LEFT'}"
                        else:
                            # Turn around to approach from farther away
                            send_velocity_ground(0, 0.8)
                            status = "OVERSHOT - TURNING AROUND"
                        color_status = (0, 165, 255)  # Orange
                    else:
                        send_velocity_ground(0, 0)
                        status = f"TOO CLOSE - WAITING ({overshoot_count}/30)"
                        color_status = (0, 0, 255)
                
                else:
                    overshoot_count = 0
                    # Normal approach behavior
                    forward_vel = pid.compute_forward(distance_error)
                    yaw_rate = pid.compute_turn(-angle_error)
                    
                    # Reduce forward speed when turning sharply
                    if abs(angle_error) > np.radians(20):
                        forward_vel *= 0.3
                    elif abs(angle_error) > np.radians(10):
                        forward_vel *= 0.6
                    
                    send_velocity_ground(forward_vel, yaw_rate)
                    
                    direction = "FWD" if forward_vel > 0 else "STOP"
                    turn_dir = "RIGHT" if yaw_rate > 0 else "LEFT" if yaw_rate < 0 else "STRAIGHT"
                    
                    status = f"{direction} {turn_dir} | D:{distance:.2f}m A:{np.degrees(angle):.1f}°"
                    color_status = (255, 255, 0)
                
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, color_status, 2)
                
            else:
                target_lost_count += 1
                overshoot_count = 0
                
                if target_lost_count > 10:
                    pid.reset()
                    
                search_state = forward_only_search(search_state)
                
                status = f"SEARCHING... {search_state['pattern']} ({target_lost_count})"
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 0, 255), 2)
            
            # Add crosshair and forward-only indicator
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2-20, h//2), (w//2+20, h//2), (255, 255, 255), 1)
            cv2.line(frame, (w//2, h//2-20), (w//2, h//2+20), (255, 255, 255), 1)
            cv2.putText(frame, "FORWARD ONLY", (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (255, 255, 255), 1)
            
            cv2.imshow('Forward-Only ArUco Approach', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        except Exception as e:
            print(f"Loop error: {e}")
            send_velocity_ground(0, 0)
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Interrupted by user")
except Exception as e:
    print(f"System error: {e}")
finally:
    print("Shutting down...")
    send_velocity_ground(0, 0)
    time.sleep(0.5)
    try:
        pipeline.stop()
    except:
        pass
    cv2.destroyAllWindows()
    master.close()
    print("Forward-only vehicle shutdown complete")
