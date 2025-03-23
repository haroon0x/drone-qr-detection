from dronekit import connect , VehicleMode , LocationGlobalRelative
import time ,socket ,argparse , sys , threading 
import cv2
from drone_control import *
from qr_detector import *

        
def main():
    vehicle = connect_copter()
    print("Connected to vehicle!")

    # saving the home location.
    home_location = vehicle.location.global_relative_frame
    
    print(f"Home Locataion : Lat - {home_location.lat} , Lon - {home_location.lon}")
    print(f" GPS: {vehicle.gps_0}")
    print(f" Battery: {vehicle.battery}")
    print(f" Mode: {vehicle.mode.name}")

    
    def on_qr_detected(data):
        print(f"QR data: {data}")
        if data:
            print("QR Code Detected - Landing on precise qr location!....")
            qr_detected.set()
            vehicle.mode = VehicleMode("LAND")

    qr_detected = threading.Event()
    stop_scanning = threading.Event()
    qr_thread = threading.Thread(target=search_for_qr_thread , args=(stop_scanning , on_qr_detected))
    qr_thread.daemon = True  # Thread will exit when main program exits
    
    try:
        print("About to takeoff")
        vehicle.mode=VehicleMode("GUIDED")
        if arm_and_takeoff(vehicle, 5):
            print("Take offsuccesfull.")
            
            # Define Waypoint
            target_lat = 47.397742
            target_lon = 8.5455993

            print("Flying to waypoint...")
            go_to(vehicle, target_lat, target_lon)

            reached_wapoint = False
            timer_start = time.time()
            timeout = 120

            
            qr_thread.start()



        #vehicle.mode = VehicleMode("LAND")
        #time.sleep(2)
        #vehicle.close()
        #print("Vehicle disconnected. \nScript Ended")

        else:
            print("Takeoff failed!")

    except Exception as e:
        print(f"Error: {e}")
        

    finally:
        stop_scanning.set()  # Signal the thread to stop
        qr_thread.join(timeout=3)  # Wait for thread to finish (with timeout)





if __name__ == "__main__":
    main()