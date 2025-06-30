#!/usr/bin/env python3

from pymavlink import mavutil
import os
import time

# Initialize connection to the flight controller (Pixhawk 6x)
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()

# State tracking
previous_state = None
last_print = time.time()

def clear_screen():
    """Clear the terminal screen in a cross-platform way."""
    os.system('cls' if os.name == 'nt' else 'clear')

def set_servo_pwm(servo_num, pwm):
    """Set servo with confirmation tracking."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo_num, pwm, 0, 0, 0, 0, 0
    )
    return f"SERVO {servo_num} → {pwm}μs"

while True:
    try:
        # Read RC channels
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        if not msg:
            time.sleep(0.1)
            continue

        ch3 = msg.chan3_raw  # Left stick (Y-Axis)
        current_state = "ON" if ch3 >= 1500 else "OFF"

        # Only update when state changes
        if current_state != previous_state:
            clear_screen()
            print("┌─────────────────────────────┐")
            print(f"│ ELECTROMAGNET: {current_state:^11}  │")
            print(f"│ CH3 Value: {ch3:4}μs           │")
            print("└─────────────────────────────┘")
            pwm = 1900 if current_state == "ON" else 1100
            confirmation = set_servo_pwm(9, pwm)
            print(f"\n{confirmation}")
            previous_state = current_state
            last_print = time.time()
        # Continuous status (every 2 seconds)
        elif time.time() - last_print > 2:
            print(f"[Status] Magnet remains {current_state} (CH3: {ch3}μs)")
            last_print = time.time()

        time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting on user request.")
        break
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
