#!/usr/bin/env python3

from pymavlink import mavutil
import os
import time

# Initialize connection to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
master.wait_heartbeat()

# Arm the rover with safety override
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # Confirmation
    1,  # Arm
    21196, 0, 0, 0, 0, 0  # Force arm parameters
)
print("Arming command sent - verify safety switches!")

# State tracking
prev_magnet_state = None
last_update = time.time()

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def set_servo_pwm(servo_num, pwm):
    """Control Aux1 (SERVO9) output"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo_num, pwm, 0, 0, 0, 0, 0
    )
    return f"Aux1/SERVO9 → {pwm}μs"

while True:
    try:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        if not msg:
            continue
            
        # Channel mapping
        steering = msg.chan1_raw  # Main3 (Steering)
        throttle = msg.chan2_raw  # Main1 (Throttle)
        magnet_ctl = msg.chan3_raw  # Aux1/SERVO9 control
        
        current_state = "ON" if magnet_ctl >= 1800 else "OFF"

        # Handle magnet state changes
        if current_state != prev_magnet_state:
            clear_screen()
            print("┌──────────────────────────────────┐")
            print(f"│ Throttle (CH1): {throttle:4}μs           │")
            print(f"│ Steering (CH2): {steering:4}μs           │")
            print(f"│ Magnet (CH3):   {current_state:^10}     │")
            print("└──────────────────────────────────┘")
            
            pwm = 1900 if current_state == "ON" else 1100
            print(f"\n{set_servo_pwm(9, pwm)}")
            
            prev_magnet_state = current_state
            last_update = time.time()
        
        # Status update every 2 seconds
        elif time.time() - last_update > 2:
            print(f"[Active] Throttle: {throttle}μs | Steering: {steering}μs")
            last_update = time.time()

        time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\nDisarming...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0  # Force disarm
        )
        break
