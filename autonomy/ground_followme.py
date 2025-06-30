from pymavlink import mavutil
import cv2
import numpy as np
import time
import pyrealsense2 as rs
import math

# MAVLink connection (verified parameters)
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()
master.set_mode('GUIDED')
time.sleep(1)  # Allow mode transition

# Tuned parameters
CRUISE_SPEED = 2.0       # m/s
TARGET_DISTANCE_M = 0.381  # 15 inches
DEADBAND_M = 0.025        # ±1 inch
FRAME_RATE = 30          # Hz
CAMERA_HEIGHT = 0.5      # meters (measure actual height)
CAMERA_TILT = 30         # degrees (camera angle from horizontal)

class PIDController:
    def __init__(self):
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.05
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:
            self.integral += error * dt
            derivative = (error - self.last_error) / dt
            self.integral = np.clip(self.integral, -1.0, 1.0)
            
            output = (self.kp * error + 
                     self.ki * self.integral + 
                     self.kd * derivative)
            
            output = np.clip(output, -CRUISE_SPEED, CRUISE_SPEED)
            self.last_error = error
            self.last_time = current_time
            return output
        return 0

def setup_realsense():
    """Robust RealSense initialization with error recovery"""
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Configure streams
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FRAME_RATE)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FRAME_RATE)
        
        # Start pipeline
        pipeline.start(config)
        
        # Warmup sensor
        for _ in range(30):
            pipeline.wait_for_frames()
        
        align = rs.align(rs.stream.color)
        print("RealSense initialized")
        return pipeline, align
        
    except Exception as e:
        print(f"Camera error: {str(e)}")
        raise

def get_frames(pipeline, align):
    """Reliable frame acquisition with timeout"""
    try:
        frames = pipeline.wait_for_frames(5000)  # 5s timeout
        aligned = align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        return color, depth
    except RuntimeError:
        return None, None

def send_velocity(vx):
    """Verified movement command from your working script"""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Velocity only
        0, 0, 0, vx, 0, 0, 0, 0, 0, 0, 0
    )

def detect_ground_square(color_frame, depth_frame):
    """Perspective-aware square detection for ground markers"""
    color_img = np.asanyarray(color_frame.get_data())
    depth_img = np.asanyarray(depth_frame.get_data())
    
    # Perspective transformation matrix
    tilt_rad = np.radians(CAMERA_TILT)
    M = cv2.getPerspectiveTransform(
        np.float32([[0,0], [640,0], [640,480], [0,480]]),
        np.float32([[0,0], [640,0], 
                   [640+300*np.sin(tilt_rad),480], 
                   [-300*np.sin(tilt_rad),480]])
    )
    
    # Warp image to compensate for camera tilt
    warped = cv2.warpPerspective(color_img, M, (640,480))
    
    # Detection in warped image
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7,7), 0)
    edges = cv2.Canny(blurred, 30, 100)
    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
        
        if len(approx) == 4:
            x,y,w,h = cv2.boundingRect(approx)
            aspect = w/h
            
            if 0.5 <= aspect <= 2.0 and cv2.contourArea(cnt) > 500:
                # Transform points back to original perspective
                inv_M = cv2.getPerspectiveTransform(
                    np.float32([[-300*np.sin(tilt_rad),480], 
                               [640+300*np.sin(tilt_rad),480], 
                               [640,0], [0,0]]),
                    np.float32([[0,480], [640,480], [640,0], [0,0]])
                )
                approx_orig = cv2.perspectiveTransform(approx.astype(np.float32), inv_M)
                
                # Get center in original image
                M = cv2.moments(approx_orig)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Calculate real horizontal distance
                depth = depth_img[cy, cx]/1000  # Convert mm to meters
                horizontal_distance = depth * math.cos(math.radians(CAMERA_TILT))
                
                # Draw results
                cv2.drawContours(color_img, [approx_orig.astype(int)], -1, (0,255,0), 2)
                return horizontal_distance, color_img
                
    return None, color_img

# Main execution
pipeline, align = setup_realsense()
pid = PIDController()

try:
    while True:
        color, depth = get_frames(pipeline, align)
        if not color or not depth:
            print("No frames - retrying...")
            time.sleep(0.1)
            continue
            
        distance, frame = detect_ground_square(color, depth)
        
        if distance is not None:
            error = distance - TARGET_DISTANCE_M
            
            if abs(error) < DEADBAND_M:
                send_velocity(0)
                status = f"HOLDING {distance:.2f}m"
                color = (0,255,0)
            else:
                velocity = pid.compute(error)
                send_velocity(velocity)
                status = f"{'FAR' if error>0 else 'NEAR'} {abs(error):.2f}m | {velocity:.1f}m/s"
                color = (0,0,255) if error>0 else (255,0,0)
            
            cv2.putText(frame, status, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        else:
            send_velocity(0)
            cv2.putText(frame, "NO TARGET", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        
        cv2.imshow('Ground Following', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    send_velocity(0)  # Stop movement
    pipeline.stop()
    cv2.destroyAllWindows()
    master.close()
    print("System shutdown")
