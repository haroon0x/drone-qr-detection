from dronekit import connect , VehicleMode , LocationGlobalRelative
from drone_utils import *
import time
import socket
import argparse
import sys

def connect_copter():
    """
    Connects to the copter from the ip passed through the terminal.
    """
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect' , required=True, 
        help="Vehicle connection string (default: '127.0.0.1:14550' - SITL simulation)" )
    args = parser.parse_args()
    connection_string = args.connect

    try:
        print(f"Connecting to vehicle on: {args.connect}")
        vehicle = connect(connection_string ,wait_ready = True , timeout= 50)
        print(f"Autopilot firmware: {vehicle.version}")
        print(f"Supported modes: {vehicle.capabilities}")
        return vehicle
    except socket.error as e:
        print(f"Connection failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Connection Failed!, Error : {e}")
        sys.exit(1)

def arm_and_takeoff(vehicle , target_altitude):
    """ Arms the vehicle and flies to target altitude in Guided Mode"""
    
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting For vehicle to become armable")
        time.sleep(2)
    

    # Ensuring that the mode is changed to GUIDED
    start = time.time()
    while vehicle.mode.name!="GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        if time.time() - start >= 20:
            print("Failed to enter GUIDED mode. Aborting takeoff.")    
            return False
        print("Waiting for vehicle to enter GUIDED mode...")
    
    print("Arming motors..")
    vehicle.armed = True
    
    # Ensuring that copter is armed
    start = time.time()
    while not vehicle.armed:
        if time.time() - start >=15:
            print("Failed to arm. Aborting Takeoff")
            return False
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    
    print(f"Taking Off to {target_altitude} meters")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Current Altitude: {current_alt}")
        if current_alt>=target_altitude*0.95:
            print("Target Altitude Reached")
            break
        time.sleep(1)
    
    return True
        
def go_to(vehicle , target_lat , target_lon , target_alt = 5):
    """
    Commands for the drone to fly to a specified latitude , longitude and altitude

    Parameters:
        vehicle (dronekit.Vehicle): The connected drone.
        target_lat (float): Target latitude.
        target_lon (float): Target longitude.
        target_alt (float, optional): Target altitude (default is 5m).
    
    Returns:
        bool: True if target is reached, False if timeout occurred.
    """
    print(f"Going to Latitude: {target_lat}, Longitude: {target_lon}, Altitude: {target_alt}")
    target_location = LocationGlobalRelative(target_lat,target_lon,target_alt)
    vehicle.simple_goto(target_location)
    start = time.time()
    timeout = 120

    last_log_time = -1
    while time.time() - start < timeout:
        current_location = vehicle.location.global_relative_frame
        dist = get_distance_metres(current_location, target_location)
    
        current_time = int(time.time() - start)
        if current_time % 5 == 0 and current_time != last_log_time:
            print(f"Distance to Target: {dist:.2f} meters")
            print(f"Current position: Lat: {current_location.lat}, Lon: {current_location.lon}")
            last_log_time = current_time
        
        if dist < 0.5:  # Considered reached when within 1 meter
            print("Reached Target Location!")
            return True

        time.sleep(1)
    
    if time.time() - start >= timeout:
        print("Timeout! Could not reach the target.")
        return False