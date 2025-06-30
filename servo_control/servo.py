from pymavlink import mavutil
import time
 
# Send-only connection to Jetson B
#master = mavutil.mavlink_connection('udpout:192.168.137.124:14550')
#master = mavutil.mavlink)connection('/dev/ttyTHS0')
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
 
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")
 
# Request AUTOPILOT_VERSION to test connection
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,  # confirmation
    0, 0, 0, 0, 0, 0, 0  # param1=0 (AUTOPILOT_VERSION), rest = 0
)
 
def set_servo_pwm(servo_instance, pwm_value):
    """Set servo PWM value using MAV_CMD_DO_SET_SERVO"""
    if pwm_value not in [1100, 1900]:
        print("Safety: Only 1100μs (demagnetize) or 1900μs (magnetize) allowed!")
        return
 
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        servo_instance,
        pwm_value,  # PWM value (strictly 1100 or 1900)
        0,0,0,0,0   # Unused parameters
    )
 
    state = "MAGNETIZED" if pwm_value == 1900 else "DEMAGNETIZED"
    print(f"Set servo {servo_instance} to {pwm_value}μS ({state})")
 
# Test sequence for electromagnet (servo9)
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
 
 
#while True:
#    msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
#    if msg:
#        print(msg)
#        break
#    else:
#        print("No response, retrying...")
