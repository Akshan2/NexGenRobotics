import time
import cv2
import serial
import DobotDllType as dType

# Load Dobot API DLL (update with correct path if needed)
api = dType.load()

# Connect to Dobot
state = dType.ConnectDobot(api, '', 115200)[0]
if state != dType.DobotConnect.DobotConnect_NoError:
    print("Failed to connect to Dobot")
    exit()
print("Connected to Dobot")

# Set initial parameters
dType.SetQueuedCmdClear(api)
dType.SetHOMEParams(api, 200, 0, 50, 0)
dType.SetHOMECmd(api, temp = 1, isQueued=1)
dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100)
dType.SetPTPCoordinateParams(api, 100, 100, 100, 100)
dType.SetPTPJumpParams(api, 10, 50)
dType.SetPTPCommonParams(api, 100, 100)

# === Define Point A ===
x_initial, y_initial, z_initial = 200, 0, 0  # Pick position (Z = 0)

# === Read Ultrasonic Sensor ===
def read_ultrasonic():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port as needed
        line = ser.readline().decode().strip()
        distance_cm = float(line)
        ser.close()
        return distance_cm
    except:
        print("Ultrasonic read failed. Using default height.")
        return 20.0  # default height in cm

# === Read LiDAR Sensor ===
def read_lidar():
    try:
        # Stub for LiDAR integration
        # Replace with your LiDAR library reading
        return 15.0  # mock value in cm
    except:
        print("LiDAR read failed. Using default height.")
        return 15.0

# === Computer Vision Integration Stub ===
def detect_object_position(frame):
    # TODO: Implement object detection logic here
    # Use OpenCV methods to detect object and return real-world X, Y, Z
    return x_initial, y_initial, z_initial

# Get dynamic z_final from sensors (lowest of the two)
ultrasonic_z = read_ultrasonic()
lidar_z = read_lidar()
z_final = min(ultrasonic_z, lidar_z)

# Define Point B (dynamic height)
x_final, y_final = 150, 100

# === Movement Sequence ===
# Move to initial pick position
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial, 0, isQueued=1)

# Close gripper (adjust values based on testing)
dType.SetEndEffectorGripper(api, enableCtrl=1, on=1, isQueued=1)
time.sleep(2)

# Lift object
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)

# Move to place position
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final, 0, isQueued=1)

# Open gripper
dType.SetEndEffectorGripper(api, enableCtrl=1, on=0, isQueued=1)
time.sleep(2)

# Retract arm
dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)

# Start execution
dType.SetQueuedCmdStartExec(api)

# === OpenCV Camera Setup (Optional) ===
# cap = cv2.VideoCapture(0)
# ret, frame = cap.read()
# if ret:
#     x_detected, y_detected, z_detected = detect_object_position(frame)
#     print("Detected object at:", x_detected, y_detected, z_detected)
# cap.release()

# Clean up (optional)
# dType.DisconnectDobot(api)

