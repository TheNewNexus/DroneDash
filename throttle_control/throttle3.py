from mavsdk import System
import asyncio
import keyboard
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError

async def control_vehicle_b():
    # Connect to Vehicle B
    drone = System()
    await drone.connect(system_address="udp://:14550")  # Vehicle B's address
 
    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to Vehicle B!")
            break
 
    # Set to MANUAL mode and arm
    print("Setting mode to MANUAL and arming...")
    # Use the flight mode enum from the correct module
    await drone.manual_control.set_manual_control_input(0, 0, 0.5, 0)  # Neutral inputs
    await drone.action.arm()
 
    # Initial PWM values (1500 = neutral)
    throttle_pwm = 1500  # Channel 1
    steer_pwm = 1500     # Channel 2
    pwm_values = [1500] * 18  # Initialize all 18 channels
 
    try:
        while True:
            # Keyboard controls (non-blocking)
            if keyboard.is_pressed('w'):
                throttle_pwm = min(1900, throttle_pwm + 10)
            elif keyboard.is_pressed('s'):
                throttle_pwm = max(1100, throttle_pwm - 10)
            if keyboard.is_pressed('a'):
                steer_pwm = max(1100, steer_pwm - 10)
            elif keyboard.is_pressed('d'):
                steer_pwm = min(1900, steer_pwm + 10)
 
            # Update channels 1 (throttle) and 2 (steering)
            pwm_values[0] = throttle_pwm  # Channel 1 (index 0)
            pwm_values[1] = steer_pwm     # Channel 2 (index 1)
 
            # Send RC override
            await drone.manual_control.send_manual_control_inputs(
                (throttle_pwm - 1500) / 400,  # Forward (-1 to 1)
                (steer_pwm - 1500) / 400,     # Right (-1 to 1)
                0.5,                          # Throttle (0 to 1)
                0                             # Yaw (-1 to 1)
            )
            
            await asyncio.sleep(0.1)  # 10Hz update rate
 
    except KeyboardInterrupt:
        # Stop and disarm
        await drone.manual_control.send_manual_control_inputs(0, 0, 0.5, 0)  # Neutral inputs
        await drone.action.disarm()
        print("\nEmergency stop and disarm!")
 
if __name__ == "__main__":
    asyncio.run(control_vehicle_b())
