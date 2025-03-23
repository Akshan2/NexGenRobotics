import time
import cv2
import serial
import threading
import tkinter as tk
from tkinter import messagebox
import DobotDllType as dType
# import pyzed.sl as sl  # ZED camera commented out

# Load Dobot API DLL (DLL must be in the same directory as this script)
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
homeCmdIndex = dType.SetHOMECmd(api, temp=1, isQueued=1)[1]
dType.SetQueuedCmdStartExec(api)

# Wait for HOMECmd to complete
print("Homing started...")
while True:
    currentIndex = dType.GetQueuedCmdCurrentIndex(api)[0]
    if currentIndex >= homeCmdIndex:
        break
    time.sleep(0.1)
print("Homing complete.")  # Wait for homing to complete
print("Setting joint params...")
dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100)
print("Joint params set.")

print("Setting coordinate params...")
dType.SetPTPCoordinateParams(api, 100, 100, 100, 100)
print("Coordinate params set.")

print("Setting jump params...")
dType.SetPTPJumpParams(api, 10, 50)
print("Jump params set.")

# Skipping SetPTPCommonParams due to known hang issue
# print("Setting common params...")
# try:
#     dType.SetPTPCommonParams(api, 50, 50)
#     print("Common params set.")
# except Exception as e:
#     print(f"Failed to set common params: {e}")
print("All home done")

# === Define Point A ===
x_initial, y_initial, z_initial = 200, 0, 0  # Pick position (Z = 0)

# === Read Ultrasonic Sensor via EIO15 (TRIG) and EIO16 (ECHO) ===
TRIG_PORT = 15  # EIO15
ECHO_PORT = 16  # EIO16

# Set EIO modes
dType.SetIOMultiplexing(api, TRIG_PORT, 0, isQueued=0)  # 0 = GPIO
dType.SetIOMultiplexing(api, ECHO_PORT, 0, isQueued=0)

def read_ultrasonic():
    try:
        dType.SetDO(api, TRIG_PORT, 1)
        time.sleep(0.00001)
        dType.SetDO(api, TRIG_PORT, 0)
        start_time = time.time()
        timeout = start_time + 0.05
        while time.time() < timeout:
            if dType.GetDI(api, ECHO_PORT)[0] == 1:
                pulse_start = time.time()
                break
        else:
            print("Ultrasonic ECHO timeout on HIGH")
            return 20.0
        timeout = time.time() + 0.05
        while time.time() < timeout:
            if dType.GetDI(api, ECHO_PORT)[0] == 0:
                pulse_end = time.time()
                break
        else:
            print("Ultrasonic ECHO timeout on LOW")
            return 20.0
        pulse_duration = pulse_end - pulse_start
        distance_cm = (pulse_duration * 34300) / 2
        return round(distance_cm, 2)
    except:
        print("Ultrasonic read failed. Using default height.")
        return 20.0

def detect_object_position_from_camera():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera")
        return x_initial, y_initial, z_initial
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Failed to read from camera")
        return x_initial, y_initial, z_initial
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = (0, 120, 70)
    upper_red = (10, 255, 255)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    moments = cv2.moments(mask)
    if moments['m00'] > 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        print(f"Object detected at pixel: ({cx}, {cy})")
    else:
        print("No object detected")
    return x_initial, y_initial, z_initial

def execute_pick_and_place():
    print("Starting pick-and-place sequence")
    ultrasonic_z = read_ultrasonic()
    print(f"Ultrasonic Z height: {ultrasonic_z} cm")
    z_final = ultrasonic_z
    x_final, y_final = 150, 100
    detect_object_position_from_camera()
    print("Queuing movement commands...")
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)
    time.sleep(0.25)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial, 0, isQueued=1)
    time.sleep(0.25)
    dType.SetEndEffectorGripper(api, enableCtrl=1, on=1, isQueued=1)
    time.sleep(0.25)
    time.sleep(2)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)
    time.sleep(0.25)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final, 0, isQueued=1)
    time.sleep(0.25)
    dType.SetEndEffectorGripper(api, enableCtrl=1, on=0, isQueued=1)
    time.sleep(0.25)
    time.sleep(2)
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)
    print("Commands sent. Pick-and-place should be executing now.")
    dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=1)  # Set gripper to rest
    

# === Manual Control GUI (Remove for autonomous version) ===
def start_gui():
    def on_run():
        threading.Thread(target=execute_pick_and_place).start()

    root = tk.Tk()
    root.title("Dobot Manual Control")
    tk.Button(root, text="KILL Program", command=lambda: (dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=0), dType.SetQueuedCmdStopExec(api), dType.SetHOMECmd(api, temp=1, isQueued=0), dType.DisconnectDobot(api), root.quit(), exit()), bg='red', fg='white', height=2, width=25).pack(pady=10)
    tk.Button(root, text="Run Pick and Place", command=on_run, height=2, width=25).pack(padx=20, pady=20)
    tk.Button(root, text="Exit", command=lambda: (root.destroy(), dType.DisconnectDobot(api))).pack(pady=10)
    root.mainloop()

print("Starting GUI...")
start_gui()
