from pymavlink import mavutil
import time
import threading
 
# === CONFIGURATION ===
# Set these for each vehicle
MY_UDP_PORT = 14550            # The port this vehicle listens on
PEER_IP = '192.168.68.77'        # The other vehicle's IP address
PEER_UDP_PORT = 14551          # The port the other vehicle listens on
 
# === MAVLink CONNECTIONS ===
# Listen for incoming messages
receiver = mavutil.mavlink_connection(f'udpin:0.0.0.0:{MY_UDP_PORT}')
 
# Send messages to the other vehicle
sender = mavutil.mavlink_connection(f'udpout:{PEER_IP}:{PEER_UDP_PORT}')
 
print("Waiting for heartbeat from peer...")
receiver.wait_heartbeat()
print(f"Heartbeat received from system {receiver.target_system}, component {receiver.target_component}")
 
# === MESSAGE RECEIVER THREAD ===
def message_handler():
    while True:
        msg = receiver.recv_match(blocking=True)
        if msg:
            print(f"\n[RECEIVED] {msg.get_type()}: {msg.to_dict()}")
 
threading.Thread(target=message_handler, daemon=True).start()
 
# === SERVO CONTROL FUNCTION ===
def set_servo_pwm(servo_instance, pwm_value):
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""
    if pwm_value not in [1100, 1900]:
        print("Safety: Only 1100μs (demagnetize) or 1900μs (magnetize) allowed!")
        return
 
    sender.mav.command_long_send(
        sender.target_system or 1,  # Fallback to 1 if unknown
        sender.target_component or 1,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        servo_instance,
        pwm_value,  # PWM value (strictly 1100 or 1900)
        0, 0, 0, 0, 0   # Unused parameters
    )
 
    state = "MAGNETIZED" if pwm_value == 1900 else "DEMAGNETIZED"
    print(f"Sent servo {servo_instance} to {pwm_value}μS ({state})")
 
# === MAIN INTERACTIVE LOOP ===
try:
    while True:
        print("\nOptions:")
        print("1: Magnetize (1900μS)")
        print("2: Demagnetize (1100μS)")
        print("q: Quit")
 
        choice = input("Select action: ").strip().lower()
 
        if choice == '1':
            set_servo_pwm(9, 1900)
        elif choice == '2':
            set_servo_pwm(9, 1100)
        elif choice == 'q':
            print("Shutting down...")
            break
        else:
            print("Invalid choice! Use 1/2/q")
 
        time.sleep(0.5)  # Allow time for command processing
 
except KeyboardInterrupt:
    print("Emergency shutdown!")
    set_servo_pwm(9, 1100)  # Fail-safe demagnetize
    print("Exiting cleanly...")
