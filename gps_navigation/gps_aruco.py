from pymavlink import mavutil
import cv2
import numpy as np
import time
import pyrealsense2 as rs
import math
import threading

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate horizontal distance between two GPS points
    Returns distance in meters
    """
    R = 6371000  # Earth radius in meters
    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)

    a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def get_gps_data():
    try:
        master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
        print("Waiting for heartbeat")
        master.wait_heartbeat()
        print("Heartbeat received")
    
        while True:
            msg_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
             
            if msg_raw:
                lat = msg_raw.lat / 1e7
                lon = msg_raw.lon / 1e7
                alt = msg_raw.alt / 1000
                satellites = msg_raw.satellites_visible

                return lat, lon, alt
    
    except KeyboardInterrupt:
        print("\nExiting GPS monitor...")
    except Exception as e:
        print(f"Error: {str(e)}")

def setup_realsense():
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        
        FRAME_RATE = 30
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
        print("RealSense initialized for ArUco detection")
        return pipeline, align, camera_matrix, dist_coeffs
        
    except Exception as e:
        print(f"Camera initialization failed: {str(e)}")
        raise

def detect_aruco_marker(color_frame, depth_frame, camera_matrix, dist_coeffs):
    MARKER_SIZE_M = 0.254
    
    # Modern OpenCV ArUco setup
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
    
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
        
        return distance, angle_rad, color_img, marker_center, True
    
    return None, None, color_img, None, False

# Global variables for ArUco detection thread
aruco_detected = False
aruco_distance = None
aruco_angle = None
aruco_frame = None
aruco_running = False

def aruco_detection_thread():
    """Continuous ArUco detection thread that runs in parallel with GPS mission"""
    global aruco_detected, aruco_distance, aruco_angle, aruco_frame, aruco_running
    
    try:
        pipeline, align, camera_matrix, dist_coeffs = setup_realsense()
        print("ArUco detection thread started")
        
        while aruco_running:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=100)
                aligned = align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                
                if not color or not depth:
                    continue
                    
                distance, angle, frame, marker_center, detected = detect_aruco_marker(
                    color, depth, camera_matrix, dist_coeffs
                )
                
                # Update global variables
                aruco_detected = detected
                aruco_distance = distance
                aruco_angle = angle
                aruco_frame = frame
                
                # Show the camera feed
                if frame is not None:
                    status_text = "MARKER DETECTED!" if detected else "SEARCHING..."
                    color = (0, 255, 0) if detected else (0, 0, 255)
                    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.8, color, 2)
                    cv2.imshow('ArUco Detection', frame)
                    cv2.waitKey(1)
                    
            except Exception as e:
                print(f"ArUco detection error: {e}")
                time.sleep(0.1)
                
    except Exception as e:
        print(f"ArUco thread initialization error: {e}")
    finally:
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()
        print("ArUco detection thread stopped")

def send_velocity_ground(master, vx, yaw_rate):
    """Send velocity command"""
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

def start_mission_with_aruco(home_waypoint_lat, home_waypoint_lon, home_waypoint_alt):
    global aruco_detected, aruco_distance, aruco_angle, aruco_running
    
    waypoint_lat = 34.4039209
    waypoint_lon = -119.6968659
    waypoint_alt = 8.04
    
    master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")
    
    # Start ArUco detection thread
    aruco_running = True
    aruco_thread = threading.Thread(target=aruco_detection_thread, daemon=True)
    aruco_thread.start()
    time.sleep(2)  # Give camera time to initialize
    
    master.set_mode_apm('GUIDED')
    time.sleep(2)
    
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(1)
    
    # Define mission items
    home_seq = 0
    home_cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    home_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    home_current = 1
    home_autocontinue = 1
    home_param = [0, 0, 0, 0, home_waypoint_lat, home_waypoint_lon, home_waypoint_alt]
    
    wp_seq = 1
    wp_cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    wp_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp_current = 0
    wp_autocontinue = 1
    wp_param = [0, 0, 0, 0, waypoint_lat, waypoint_lon, waypoint_alt]
    
    master.mav.mission_count_send(master.target_system, master.target_component, 2)
    time.sleep(0.5)
    
    master.mav.mission_item_send(
        master.target_system, master.target_component,
        home_seq, home_frame, home_cmd, home_current, home_autocontinue,
        *home_param
    )
    time.sleep(0.5)
    
    master.mav.mission_item_send(
        master.target_system, master.target_component,
        wp_seq, wp_frame, wp_cmd, wp_current, wp_autocontinue,
        *wp_param
    )
    time.sleep(0.5)
    
    while True:
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=5)
        if msg:
            print(f"Mission ACK received: type={msg.type}")
            break
    
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed!")
    
    master.set_mode_apm('AUTO')
    print("Mission started in AUTO mode with ArUco detection running...")
    
    # Monitor mission progress with ArUco detection
    TARGET_RADIUS = 15.0  # Larger radius since we're using ArUco for final approach
    STOP_SPEED_THRESHOLD = 0.1
    TIMEOUT = 300
    start_time = time.time()

    try:
        while True:
            # Check for ArUco marker detection first
            if aruco_detected and aruco_distance is not None:
                print(f"ARUCO MARKER DETECTED! Distance: {aruco_distance:.2f}m")
                print("Switching to GUIDED mode and stopping vehicle...")
                
                # Switch to GUIDED mode and stop
                master.set_mode_apm('GUIDED')
                time.sleep(1)
                send_velocity_ground(master, 0, 0)  # Stop the vehicle
                
                print("Vehicle stopped due to ArUco marker detection!")
                aruco_running = False  # Stop the ArUco thread
                return True
            
            # Continue GPS mission monitoring
            current_lat, current_lon, current_alt = get_gps_data()
            
            msg_speed = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            current_speed = msg_speed.groundspeed if msg_speed else 0.0
            
            horiz_distance = haversine(current_lat, current_lon, waypoint_lat, waypoint_lon)
            alt_diff = abs(current_alt - waypoint_alt)
            
            print(f"GPS Distance: {horiz_distance:.2f}m | Alt diff: {alt_diff:.2f}m | Speed: {current_speed:.2f}m/s | ArUco: {'DETECTED' if aruco_detected else 'SEARCHING'}")
            
            # Check GPS arrival conditions (backup)
            if current_speed <= STOP_SPEED_THRESHOLD:
                if horiz_distance <= TARGET_RADIUS and alt_diff <= 2.0:
                    print("Vehicle reached GPS target area")
                    # Continue running to wait for ArUco detection
                    pass
            
            # Timeout check
            if time.time() - start_time > TIMEOUT:
                print("Mission timeout!")
                aruco_running = False
                return False
            
            time.sleep(1)
            
    except Exception as e:
        print(f"Mission error: {e}")
        aruco_running = False
        return False

def set_servo_pwm_low():
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""
    master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        9,
        1100,
        0,0,0,0,0
    )

def set_servo_pwm_high():
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""
    master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print("Heartbeat received")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        9,
        1900,
        0,0,0,0,0
    )

if __name__ == "__main__":
    print("=== GPS MISSION WITH ARUCO DETECTION SYSTEM ===")
    
    try:
        print("MAIN LOG: Getting Home Location")
        home_lat, home_lon, home_alt = get_gps_data()
        print(f"Home Location - Lat: {home_lat}, Lon: {home_lon}, Alt: {home_alt}")

        print("MAIN LOG: STARTING GPS MISSION WITH ARUCO DETECTION")
        mission_success = start_mission_with_aruco(home_lat, home_lon, home_alt)
        
        if mission_success:
            print("MAIN LOG: MISSION COMPLETE - Vehicle stopped by ArUco detection")
        else:
            print("MAIN LOG: MISSION FAILED")
            
    except KeyboardInterrupt:
        print("\nSystem interrupted by user")
        aruco_running = False
    except Exception as e:
        print(f"System error: {e}")
        aruco_running = False
    finally:
        print("\n=== SYSTEM SHUTDOWN COMPLETE ===")
