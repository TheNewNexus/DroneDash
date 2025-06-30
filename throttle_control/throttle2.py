from pymavlink import mavutil
import time
 
# === CONNECTION ===
controller = mavutil.mavlink_connection('udp:0.0.0.0:14551')  # Vehicle B's IP
print("Waiting for heartbeat...")
controller.wait_heartbeat()
print("Heartbeat received")

 
# === ROVER-SPECIFIC MOVEMENT ===
def set_manual_control(speed, steering):
    """Speed: -1000 (reverse) to 1000 (forward)
       Steering: -1000 (left) to 1000 (right)"""
    controller.mav.manual_control_send(
        controller.target_system,
        int(speed),
        int(steering),
        0,  # Throttle (unused)
        0,  # Rotation (unused)
        0   # Buttons
    )
 
# === MAIN LOOP ===
try:
    while True:
        print("\nControls: W-forward S-stop A-left D-right Q-quit")
        cmd = input("> ").lower()
 
        if cmd == 'w': set_manual_control(500, 0)    # Forward
        elif cmd == 's': set_manual_control(0, 0)    # Stop
        elif cmd == 'a': set_manual_control(300, -500)  # Left turn
        elif cmd == 'd': set_manual_control(300, 500)   # Right turn
        elif cmd == 'q': break
        time.sleep(0.1)
 
except KeyboardInterrupt:
    set_manual_control(0, 0)
    print("\nStopped")
