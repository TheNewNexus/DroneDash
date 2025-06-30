from pymavlink import mavutil
import time

# Connect to Pixhawk - common Jetson serial ports: /dev/ttyACM0 or /dev/ttyTHS1
CONNECTION_STRING = '/dev/ttyTHS1'  # Try THS1 if ACM0 fails
BAUD_RATE = 921600  # Common Pixhawk baud rates: 57600 or 115200

def get_gps_data():
    try:
        # Establish connection
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        
        # Wait for initial heartbeat
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"Connected to system {master.target_system}, component {master.target_component}")

        # Main loop
        while True:
            # Get both raw and processed GPS data
            msg_raw = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
            msg_global = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            
            if msg_raw:
                # Convert scaled integers to decimal degrees
                lat = msg_raw.lat / 1e7
                lon = msg_raw.lon / 1e7
                alt = msg_raw.alt / 1000  # Meters
                satellites = msg_raw.satellites_visible
                print(f"RAW: Lat:{lat:.6f}°, Lon:{lon:.6f}°, Alt:{alt:.1f}m, Sats:{satellites}")

            if msg_global:
                # Processed global position (relative to home)
                rel_alt = msg_global.relative_alt / 1000  # Millimeters to meters
                print(f"GLOBAL: Relative Altitude: {rel_alt:.1f}m")
            
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting GPS monitor...")
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    get_gps_data()
