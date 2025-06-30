import cv2
import pyrealsense2 as rs

# Create pipeline object
pipeline = rs.pipeline()

# Configure pipeline
config = rs.config()
config.enable_device("F123456789") # Replace with your camera serial number
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Loop until the user presses a key
try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()

        # Get depth and color frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert frames to OpenCV images
        if not depth_frame or not color_frame:
            continue
        depth_image = cv2.cvtColor(depth_frame.get_data(), cv2.COLOR_GRAY2BGR)
        color_image = cv2.cvtColor(color_frame.get_data(), cv2.COLOR_BGR2RGB)

        # Display images
        cv2.imshow("Depth Image", depth_image)
        cv2.imshow("Color Image", color_image)

        # Press ESC to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
