from pymavlink import mavutil
import cv2
import numpy as np
import time
import pyrealsense2 as rs

# MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()

print("Setting GUIDED mode...")
master.set_mode('GUIDED')
time.sleep(2)

# Parameters for GPS-to-ArUco transition
SCAN_SPEED = 0.15        # Slow scanning speed
APPROACH_SPEED = 0.25    # Approach speed once marker found
TARGET_DISTANCE_M = 0.8  # Final distance from marker
MARKER_SIZE_M = 0.254    # 10 inch marker
SCAN_RADIUS_M = 3.0      # How far to scan from GPS point

# ArUco setup
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

class GPSToArucoController:
    def __init__(self):
        self.state = "INITIAL_SCAN"  # INITIAL_SCAN, SPIRAL_SEARCH, MARKER_FOUND, APPROACHING, COMPLETE
        self.scan_start_time = time.time()
        self.scan_pattern_index = 0
        self.spiral_leg = 0
        self.spiral_direction = 1  # 1 for right, -1 for left
        self.marker_lost_count = 0
        
    def get_scan_pattern(self):
        """Systematic scanning pattern around GPS drop point"""
        patterns = [
            # Phase 1: Look around in place (360° scan)
            ("turn_scan", [(0, 0.4, 8), (0, -0.4, 8), (0, 0.4, 8), (0, -0.4, 8)]),
            
            # Phase 2: Move and scan pattern
            ("move_scan", [
                (SCAN_SPEED, 0, 3),      # Forward
                (0, 0.6, 4),             # Turn right
                (SCAN_SPEED, 0, 3),      # Forward
                (0, 0.6, 4),             # Turn right
                (SCAN_SPEED, 0, 3),      # Forward
                (0, 0.6, 4),             # Turn right
                (SCAN_SPEED, 0, 3),      # Forward
                (0, 0.6, 4),             # Turn right (complete square)
            ]),
            
            # Phase 3: Expanding spiral search
            ("spiral_search", "dynamic")
        ]
        return patterns

def setup_realsense():
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Optimized for ground scanning
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        profile = pipeline.start(config)
        
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        
        camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                 [0, intrinsics.fy, intrinsics.ppy],
                                 [0, 0, 1]])
        
        dist_coeffs = np.array(intrinsics.coeffs)
        
        # Warmup
        for _ in range(30):
            pipeline.wait_for_frames()
        
        align = rs.align(rs.stream.color)
        print("RealSense initialized for GPS-to-ArUco transition")
        return pipeline, align, camera_matrix, dist_coeffs
        
    except Exception as e:
        print(f"Camera setup failed: {e}")
        return None, None, None, None

def send_velocity(vx, yaw_rate):
    """Send movement command (forward-only vehicle)"""
    try:
        # Ensure no reverse movement
        vx = max(0, vx)
        
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0, vx, 0, 0, 0, 0, 0, 0, yaw_rate
        )
        return True
    except Exception as e:
        print(f"Movement command failed: {e}")
        return False

def detect_aruco_ground(color_frame, depth_frame, camera_matrix, dist_coeffs):
    """Enhanced ArUco detection optimized for ground scanning"""
    color_img = np.asanyarray(color_frame.get_data())
    depth_img = np.asanyarray(depth_frame.get_data())
    
    # Enhanced preprocessing for ground markers
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    
    # Adaptive histogram equalization for varying lighting
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    
    # Noise reduction
    gray = cv2.medianBlur(gray, 3)
    
    # Detect markers
    corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Draw all detected markers
        cv2.aruco.drawDetectedMarkers(color_img, corners, ids)
        
        # Find the best marker (largest area = closest/most reliable)
        best_marker_idx = 0
        best_area = 0
        
        for i, corner in enumerate(corners):
            area = cv2.contourArea(corner)
            if area > best_area:
                best_area = area
                best_marker_idx = i
        
        # Use best marker for pose estimation
        best_corner = [corners[best_marker_idx]]
        best_id = [ids[best_marker_idx]]
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            best_corner, MARKER_SIZE_M, camera_matrix, dist_coeffs
        )
        
        marker_id = best_id[0][0]
        rvec = rvecs[0]
        tvec = tvecs[0]
        
        # Draw coordinate frame
        cv2.drawFrameAxes(color_img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
        
        # Calculate distance and angle
        distance = np.linalg.norm(tvec)
        angle_rad = np.arctan2(tvec[0][0], tvec[0][2])
        
        # Comprehensive status overlay
        cv2.putText(color_img, f"MARKER FOUND! ID: {marker_id}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(color_img, f"Distance: {distance:.2f}m", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(color_img, f"Angle: {np.degrees(angle_rad):.1f}°", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(color_img, f"Area: {int(best_area)}", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return distance, angle_rad, color_img, True
    
    # Show scanning status when no marker found
    cv2.putText(color_img, "SCANNING FOR MARKER...", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(color_img, f"Rejected: {len(rejected)}", (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    return None, None, color_img, False

def execute_scanning_state(controller):
    """Execute systematic scanning around GPS drop point"""
    current_time = time.time()
    elapsed = current_time - controller.scan_start_time
    
    if controller.state == "INITIAL_SCAN":
        # Phase 1: 360° rotation scan in place
        if elapsed < 20:  # 20 seconds of turning
            send_velocity(0, 0.4)  # Turn right slowly
            return "INITIAL SCAN - Rotating 360°", (255, 165, 0)
        else:
            controller.state = "SPIRAL_SEARCH"
            controller.scan_start_time = current_time
            controller.spiral_leg = 0
            return "INITIAL SCAN - Complete", (255, 165, 0)
    
    elif controller.state == "SPIRAL_SEARCH":
        # Phase 2: Expanding spiral search pattern
        leg_duration = 2 + (controller.spiral_leg * 0.5)  # Increasing leg length
        
        if elapsed < leg_duration:
            if controller.spiral_leg % 2 == 0:  # Even legs: move forward
                send_velocity(SCAN_SPEED, 0)
                direction = "Forward"
            else:  # Odd legs: turn
                yaw_rate = 0.6 * controller.spiral_direction
                send_velocity(0, yaw_rate)
                direction = f"Turn {'Right' if controller.spiral_direction > 0 else 'Left'}"
            
            return f"SPIRAL SEARCH - Leg {controller.spiral_leg}: {direction}", (255, 0, 255)
        else:
            # Move to next leg
            controller.spiral_leg += 1
            controller.scan_start_time = current_time
            
            # Change direction every 2 legs (after forward + turn)
            if controller.spiral_leg % 2 == 0:
                controller.spiral_direction *= -1
            
            # Limit spiral size
            if controller.spiral_leg > 12:  # Reset after 6 complete legs
                controller.state = "INITIAL_SCAN"
                controller.scan_start_time = current_time
                controller.spiral_leg = 0
                return "SPIRAL SEARCH - Restarting", (255, 0, 255)
            
            return f"SPIRAL SEARCH - Starting Leg {controller.spiral_leg}", (255, 0, 255)
    
    return "UNKNOWN SCAN STATE", (255, 255, 255)

def execute_approach(distance, angle):
    """Execute approach to marker once found"""
    distance_error = distance - TARGET_DISTANCE_M
    
    # At target distance
    if abs(distance_error) < 0.1 and abs(angle) < np.radians(10):
        send_velocity(0, 0)
        return "MISSION COMPLETE!", (0, 255, 0)
    
    # Need to align first
    elif abs(angle) > np.radians(15):
        yaw_rate = 0.5 if angle > 0 else -0.5
        send_velocity(0, yaw_rate)
        return f"ALIGNING - Turn {'Right' if angle > 0 else 'Left'}", (255, 255, 0)
    
    # Approach with minor corrections
    else:
        forward_vel = min(APPROACH_SPEED, distance_error * 0.8) if distance_error > 0 else 0
        yaw_rate = np.clip(angle * -0.6, -0.3, 0.3)
        
        send_velocity(forward_vel, yaw_rate)
        
        if distance_error > 0:
            return f"APPROACHING - {distance:.2f}m to go", (0, 255, 255)
        else:
            return f"TOO CLOSE - Repositioning", (255, 165, 0)

# Main execution
def main():
    pipeline, align, camera_matrix, dist_coeffs = setup_realsense()
    if pipeline is None:
        print("Failed to initialize camera")
        return
    
    controller = GPSToArucoController()
    
    print("="*50)
    print("GPS-TO-ARUCO TRANSITION SYSTEM")
    print("="*50)
    print("This script assumes:")
    print("1. GPS has brought you near the marker location")
    print("2. You're within ~3m of the actual marker")
    print("3. The 10x10 inch ArUco marker is on the ground")
    print("4. Vehicle can only move forward (no reverse)")
    print("="*50)
    print("Starting systematic area scan...")
    print("Press 'q' to quit")
    
    try:
        while True:
            # Get camera frames
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            aligned = align.process(frames)
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            
            if not color or not depth:
                continue
            
            # Detect ArUco marker
            distance, angle, frame, marker_found = detect_aruco_ground(
                color, depth, camera_matrix, dist_coeffs
            )
            
            if marker_found:
                # Marker detected - switch to approach mode
                controller.marker_lost_count = 0
                controller.state = "MARKER_FOUND"
                
                status, color_status = execute_approach(distance, angle)
                
            else:
                # No marker - continue scanning
                controller.marker_lost_count += 1
                
                # If we had the marker but lost it, try to reacquire
                if controller.state == "MARKER_FOUND" and controller.marker_lost_count < 30:
                    send_velocity(0, 0)  # Stop and wait
                    status = f"MARKER LOST - Waiting ({controller.marker_lost_count}/30)"
                    color_status = (0, 165, 255)
                else:
                    # Execute scanning pattern
                    if controller.marker_lost_count >= 30:
                        controller.state = "SPIRAL_SEARCH"  # Resume searching
                        controller.marker_lost_count = 0
                    
                    status, color_status = execute_scanning_state(controller)
            
            # Display comprehensive status
            cv2.putText(frame, f"State: {controller.state}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_status, 2)
            cv2.putText(frame, status, (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_status, 2)
            
            # Add scanning indicator
            h, w = frame.shape[:2]
            cv2.circle(frame, (w//2, h//2), 10, (255, 255, 255), 2)
            cv2.putText(frame, "GPS->ArUco Transition", (10, h-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('GPS to ArUco Transition', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Shutting down...")
        send_velocity(0, 0)
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()
        master.close()
        print("GPS-to-ArUco transition complete")

if __name__ == "__main__":
    main()
