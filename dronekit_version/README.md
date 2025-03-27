# Drone QR Detection

This project enables **autonomous drone navigation and precision landing** using GPS waypoints and **QR code detection** for enhanced accuracy. The drone takes off, navigates to a specified location, scans for a QR code, and lands precisely on it.

## **Features**  
✅ Autonomous takeoff and navigation to a GPS waypoint  
✅ Real-time QR code detection for precision landing  
✅ Uses **`pymavlink`** for drone control  
✅ Vision-based final landing using OpenCV  
✅ Threaded QR scanning for real-time detection  

## Project Structure
```
 │── README.md
 │── pyproject.toml
 │── uv.lock
 │── .python-version
 │
 ├── src/
 │   ├── main.py              # Main entry point of the application
 │   ├── config.py            # Configuration settings
 │   ├── qr_detector.py       # QR code detection logic
 │   ├── drone_control.py     # Core drone control functions (e.g., takeoff, landing)
 │   ├── drone_utils.py       # Utility functions (e.g., distance calculation)
```

## **Installation**  

### Prerequisites
- `Python 3.12`
- `pymavlink` for drone communication
- `opencv-python` for QR detection


### Setup
1. Clone the repository:
   ```sh
   git clone https://github.com/haroon0x/drone-qr-detection.git
   cd drone-qr-detection
   ```
2. Install dependencies using `uv`: 
   ```sh
   pip install uv
   uv pip install
   ```
3. Connect to the drone:
   ```sh
   python src/main.py --connect 127.0.0.1:14550
   ```
For a real drone, replace `127.0.0.1:14550` with your **actual telemetry connection string**.

## **Usage**  
1. **Start the script**:  
   ```sh
   python main.py --connect 127.0.0.1:14550
   ```
2. The drone will:  
   - Take off  
   - Navigate to the **specified GPS location**  
   - Scan for a **QR code**  
   - Land **precisely on the QR code**  


## **Function Overview**  
### `arm_and_takeoff(vehicle, altitude)`  
- Arms the drone and ascends to the given **altitude**  

### `go_to(vehicle, lat, lon)`  
- Moves the drone to the given **GPS coordinates**  

### `search_for_qr_thread(stop_event, callback)`  
- Runs a **separate thread** for QR code detection  
- Calls `callback(data)` when a QR is detected  

### `on_qr_detected(data)`  
- Handles QR detection and **initiates landing**  

## Acknowledgments
- Built using DroneKit and OpenCV.
- Inspired by precision landing research in autonomous drones