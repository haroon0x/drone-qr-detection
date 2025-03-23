import cv2
import numpy as np
import threading
import time 

def detect_qr(frame , qr_detector):
    """
    Detect and decode a QR code from the provided frame.
    Parameters:
        frame (numpy.ndarray): The input image in BGR format.
        qr_detector (cv2.QRCodeDetector): QR code detector object

    Returns:
        data (str): The decoded QR code data (UTF-8 string), or empty if none detected.
        bbox (numpy.ndarray): The quadrangle (bounding box) of the detected QR code.
        straight_qr_img (numpy.ndarray): A rectified image of the QR code(Frontal view).

    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data, bbox, straight_qr_img = qr_detector.detectAndDecode(gray_frame)

    if bbox is None or len(bbox) ==0:   
        _, thresholded_frame = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY)
        blurred_frame = cv2.GaussianBlur(thresholded_frame, (5, 5), 0)
        data, bbox, straight_qr_img = qr_detector.detectAndDecode(blurred_frame)

    return data, bbox , straight_qr_img

def search_for_qr_thread(stop_event, callback = None):
    """ Thread which searches for qr.

    Parameters:
        stop_event (threading.Event): Event to signal when to stop
        callback (function,optional): Function to call when QR code is detected
    """
    cap = cv2.VideoCapture(0) 
    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return
    
    qr_detector = cv2.QRCodeDetector()
    
    while not stop_event.is_set():
        ret, frame = cap.read()  # ret - boolean value indicating capturing of frame.
        if not ret:
            print("Failed to capture video. Retrying")
            time.sleep(1)
            continue
        
        data , bbox , _ = detect_qr(frame , qr_detector)

        if bbox is not None and len(bbox) > 0:
            # Convert bbox to appropriate integer points
            bbox = bbox.astype(int)
            # Draw bounding box lines around the QR code
            cv2.polylines(frame, [bbox], True, (0, 255, 0), 2)
            if data:
                print(f"Qr Data : {data}")
                cv2.putText(frame, f"QR Code: {data}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
                if callback and callable(callback):
                    callback(data)


        cv2.imshow("QR Code Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

