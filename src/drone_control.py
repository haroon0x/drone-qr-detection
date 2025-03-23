from dronekit import connect , VehicleMode , LocationGlobalRelative
import time
import socket
import argparse
import sys

def connect_copter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect' , required=True, 
        help="Vehicle connection string (default: '127.0.0.1:14550' - SITL simulation)",
        default= '127.0.0.1:14550' )
    args = parser.parse_args()
    connection_string = args.connect

    try:
        print(f"Conntecting to vehicle on: {args.connect}")
        vehicle = connect(connection_string ,wait_ready = True , timeout= 50)
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
    while not vehicle.is_armable():
        print("Waiting For vehicle to become armable")
        time.sleep(1.2)
    
    print("Setting mode to GUIDED")
    vehicle.mode = VehicleMode("GUIDED")

    # Ensuring that the mode is changed to GUIDED
    start = time.time()
    while vehicle.mode.name!="GUIDED":
        if time.time() - start >= 10:
            print("Failed to enter GUIDED mode. Aborting takeoff.")
            return False
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    
    print("Arming motors..")
    vehicle.armed = True
    
    # Ensuring that copter is armed
    start = time.time()
    while not vehicle.armed:
        if time.time() - start >=10:
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
    
    """
    print(f"Going to Latitude: {target_lat}, Longitude: {target_lon}, Altitude: {target_alt}")
    target_location = LocationGlobalRelative(lat= target_lat,lon=target_lon,alt=target_alt)
    vehicle.simple_goto(target_location)
