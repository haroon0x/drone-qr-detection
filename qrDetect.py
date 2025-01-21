import collections
import sys

# Monkey patch for Python 3.10+ to handle MutableMapping
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import cv2
import numpy as np


# Connect to the vehicle
connection_string = 'tcp:127.0.0.1:5760'
#connection_string = 'COM3'
baud_rate = 112500     

print(f"Connecting to vehicle on: {connection_string}...")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

print("Connected to vehicle!")
print(f" GPS: {vehicle.gps_0}")
print(f" Battery: {vehicle.battery}")
print(f" Mode: {vehicle.mode.name}")

cap = cv2.VideoCapture(0) 
qr_detector = cv2.QRCodeDetector()

print("Press 'q' to exit the video feed.")

try:
    while True:
        ret, frame = cap.read()


        if not ret:
            print("Failed to capture video. Exiting...")
            break

        # Preprocessing: Convert to grayscale, threshold, and blur
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresholded_frame = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY)
        blurred_frame = cv2.GaussianBlur(thresholded_frame, (5, 5), 0)


        data, bbox, straight_qr_img = qr_detector.detectAndDecode(blurred_frame)

        if bbox is not None and len(bbox) > 0:
            for i in range(len(bbox)):
                pt1 = tuple(map(int, bbox[i][0]))
                pt2 = tuple(map(int, bbox[(i + 1) % len(bbox)][0]))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:

                print(f"Qr Data : {data}")
                cv2.putText(frame, f"QR Code: {data}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # Command the vehicle to land
                if vehicle.mode.name != "LAND":
                    print("QR Code detected. Initiating landing...")
                    vehicle.mode = VehicleMode("LAND")


        cv2.imshow("QR Code Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Keyboard Interrupted......")


cap.release()
cv2.destroyAllWindows()



# Close the vehicle connection
if vehicle:
    vehicle.close()
    print("Vehicle connection closed.")