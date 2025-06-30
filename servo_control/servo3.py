from pymavlink import mavutil
import time
import threading
import sys
from inputimeout import inputimeout, TimeoutOccurred
 
# === CONFIGURATION ===
VEHICLE_ID = 1  # 1 for first vehicle, 2 for second
MY_PORT = 14550 if VEHICLE_ID == 1 else 14551
PEER_IP = '192.168.68.77'  # Replace with peer's actual IP
PEER_PORT = 14551 if VEHICLE_ID == 1 else 14550
 
# === MAVLink CONNECTIONS ===
conn = mavutil.mavlink_connection(
    f'udpin:0.0.0.0:{MY_PORT}',
    source_system=VEHICLE_ID,
    input=True
)
 
# === STATUS TRACKING ===
last_heartbeat = {}
current_state = "UNKNOWN"
 
def update_display():
    """Clear menu line and re-draw"""
    sys.stdout.write('\r\033[K')  # Clear current line
    print(f"\n--- Vehicle {VEHICLE_ID} [{current_state}] ---")
    print("1: Magnetize | 2: Demagnetize | q: Quit")
    sys.stdout.write("> ")
 
# === MESSAGE HANDLING ===
def handle_message(msg):
    global current_state
    if msg.get_type() == 'HEARTBEAT':
        system_id = msg.get_srcSystem()
        if system_id not in last_heartbeat:
            print(f"\n[SYSTEM {system_id} CONNECTED]")
        last_heartbeat[system_id] = time.time()
 
    elif msg.get_type() == 'COMMAND_ACK':
        if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            result = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            new_state = "MAGNETIZED" if current_state == "DEMAGNETIZED" else "DEMAGNETIZED"
            print(f"\n[STATUS] {new_state} ({result})")
            current_state = new_state
            update_display()
 
def message_receiver():
    while True:
        msg = conn.recv_match(blocking=True)
        if msg:
            handle_message(msg)
 
# === HEARTBEAT MANAGEMENT ===
def heartbeat_handler():
    while True:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0
        )
        time.sleep(1)
 
# === SERVO CONTROL ===
def send_servo_command(pwm_value):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 9, pwm_value, 0,0,0,0,0
    )
 
# === MAIN LOOP ===
if __name__ == "__main__":
    # Start threads
    threading.Thread(target=heartbeat_handler, daemon=True).start()
    threading.Thread(target=message_receiver, daemon=True).start()
 
    # Initial state
    current_state = "DEMAGNETIZED"
 
    try:
        while True:
            update_display()
            try:
                choice = inputimeout(timeout=2).lower()
                if choice == '1':
                    send_servo_command(1900)
                elif choice == '2':
                    send_servo_command(1100)
                elif choice == 'q':
                    print("\nExiting...")
                    sys.exit(0)
            except TimeoutOccurred:
                continue
 
    except KeyboardInterrupt:
        print("\nEMERGENCY SHUTDOWN!")
        send_servo_command(1100)
        sys.exit(0)
