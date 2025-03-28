from pymavlink import mavutil
from config import *
import time
    
def connect_to_drone():
    print(f'🔌 Connecting to drone on {COM_PORT}...')
    attempts = 3 

    while attempts > 0:
        try:
            master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
            master.wait_heartbeat()
            print(f'✅ Drone connected. Heartbeat from system {master.target_system}, component {master.target_component}')
            return master

        except Exception as e:
            print(f'⚠ Warning: Connection failed ({attempts} attempts left). Error: {e}')
            attempts -= 1
            time.sleep(2)

    print('❌ Error: Unable to connect to drone after multiple attempts.')
    return None 

def go_to(master, lat, lon, alt, ack_timeout=5, arrival_timeout=60, threshold=0.00005):
    """
    Goes to the specified waypoint.
    Sends a MAVLink command to move the drone to a specific latitude, longitude, and altitude.

    :param master: MAVLink connection object
    :param lat: Target latitude (decimal degrees)
    :param lon: Target longitude (decimal degrees)
    :param alt: Target altitude (meters)
    :param ack_timeout: Timeout in seconds for command acknowledgment
    :param arrival_timeout: Timeout in seconds to wait for waypoint arrival
    :param threshold: Difference threshold to determine arrival at the waypoint
    """
    if not master:
        print("❌ Error: No connection to drone.")
        return

    print(f'📍 Sending drone to → Lat: {lat}, Lon: {lon}, Alt: {alt}m')

    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # Confirmation
            0,  # Hold time (seconds)
            0,  # Accept radius (meters)
            0,  # Pass radius (meters)
            0,  # Yaw angle (degrees)
            lat,  # Latitude (decimal degrees)
            lon,  # Longitude (decimal degrees)
            alt   # Altitude (meters)
        )
    except Exception as e:
        print(f"❌ Failed to send waypoint command: {e}")
        return

    # Wait for command acknowledgment with a timeout
    ack = None
    ack_start_time = time.time()
    while time.time() - ack_start_time < ack_timeout:
        ack = master.recv_match(type='COMMAND_ACK', blocking=False)
        if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            break
        time.sleep(0.1)

    if ack is None:
        print("❌ Timeout waiting for navigation command acknowledgment.")
        return

    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ Navigation command accepted.")
    else:
        print(f"⚠️ Navigation command failed. Result: {ack.result}")
        return

    # Monitor GPS position to check arrival at the waypoint with a timeout
    pos_start_time = time.time()
    while time.time() - pos_start_time < arrival_timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            try:
                current_lat = msg.lat / 1e7  # Convert from integer degrees
                current_lon = msg.lon / 1e7
                current_alt = msg.alt / 1000  # Convert from mm to meters
            except Exception as e:
                print(f"Error processing position message: {e}")
                continue

            print(f'📡 Current Position → Lat: {current_lat}, Lon: {current_lon}, Alt: {current_alt}m')

            if abs(current_lat - lat) < threshold and abs(current_lon - lon) < threshold:
                print("✅ Drone reached the waypoint!")
                return

        time.sleep(1)

    print("❌ Timeout waiting for drone to reach the waypoint.")


def set_guided_mode(master):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 represents GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

def perform_servo_actions(master):
    set_servo(master, SERVO_CHANNEL, SERVO_PWM_FORWARD)
    time.sleep(2)
    set_servo(master, SERVO_CHANNEL, SERVO_PWM_CLOSE)
    time.sleep(2)
    set_servo(master, SERVO_CHANNEL, SERVO_PWM_STOP)
    time.sleep(5)

def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')


def arm_drone(master):
    print('🕹️ Arming drone...')

    # Send arm command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Arm command sent. Waiting for confirmation...')
    time.sleep(2)

    # Wait for arming confirmation
    for _ in range(10):  # Check for up to 10 seconds
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
        )
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('✅ Drone confirmed as armed.')
            return  # Exit function if armed

        print('⏳ Waiting for drone to arm...')
        time.sleep(1)

    print('❌ Drone failed to confirm arming. Check logs for issues.')


def takeoff_drone(master, altitude=5):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    print('⏳ Takeoff in progress...')

    time.sleep(10)  # Wait for the drone to reach the altitude
    print(f'✅ Drone has taken off to {altitude} meters.')


def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')

def land_at_coordinates(drone):
    print('🛬 Sending LAND command to given coordinates...')
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, drone.target_system, drone.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
    ))

    drone.mav.command_long_send(
        drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')
    time.sleep(10)


def set_waypoint(master, latitude, longitude, altitude):
    print(f'📍 Sending waypoint to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}m')

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,  # Time_boot_ms (set to 0 for now)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000),  # Control position, no yaw or velocity
        int(latitude * 1e7),
        int(longitude * 1e7),
        altitude,
        0, 0, 0,  # Velocity X, Y, Z
        0, 0, 0,  # Acceleration X, Y, Z
        0.0,      # Yaw
        0.0       # Yaw Rate (This was missing)
    ))

    print('✅ Waypoint command sent.')


# Arm and Takeoff
'''def arm_and_takeoff(master, altitude=5):
    print('🕹️ Arming drone...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Drone armed.')

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    time.sleep(10)'''