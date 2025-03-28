from pymavlink import mavutil
import numpy as np
import time ,socket ,argparse , sys , threading 
from drone_control import *
from qr_detector import *
from config import *


def main():
    flag=0
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    
    if not cap.isOpened():
        print('❌ ⚠ Warning: Unable to open video stream. Retrying in 5 seconds...')
        time.sleep(2)
        cap = cv2.VideoCapture(VIDEO_SOURCE) 
        if not cap.isOpened():
            print('❌ Error: Video stream unavailable. Running without video.')

    qr_detector = cv2.QRCodeDetector()
    drone = connect_to_drone()
    print("flag is: ", flag)

    if not drone:
        print('❌ Error: Unable to connect to drone after multiple attempts.')
        return
    else:
        flag=1
        
    while flag:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue
        
        detect_qr(frame , qr_detector)
        data, bbox, _ = detect_qr(frame , qr_detector)
    
        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                land_at_coordinates(drone)
                print('⏳ Waiting for 2 seconds after landing...')
                time.sleep(2)

                print('🔄 Rotating servo forward and back...')
                perform_servo_actions(drone)
                set_guided_mode(drone)
                arm_drone(drone)
                takeoff_drone(drone, altitude=3)
                go_to(drone , LATITUDE , LONGITUDE , ALTITUDE)
                land_at_coordinates(drone)
                flag=0

        cv2.imshow('QR Code Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if drone:
        drone.close()
        print("flag is: ", flag)
        print('🔌 Drone connection closed.')


if __name__ == '__main__':
    main()
