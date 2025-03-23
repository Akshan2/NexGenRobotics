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

# Move to home position after homing
index = dType.SetHOMECmd(api, temp=1, isQueued=1)[1]
while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
    time.sleep(0.1)

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
x_initial, y_initial, z_initial = 200, 0, 50  # Pick position (Z = 0)

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
            return 7.94  # Default to 3 1/8 inches in cm if ultrasonic fails
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
    cap = cv2.VideoCapture(1)  # Try camera index 1 for Dobot camera
    if not cap.isOpened():
        print("Failed to open camera")
        return x_initial, y_initial, z_initial
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Failed to read from camera")
        return x_initial, y_initial, z_initial
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] > 0:
          cx = int(M['m10'] / M['m00'])
          cy = int(M['m01'] / M['m00'])
        print(f"Object detected at pixel: ({cx}, {cy})")
    else:
        print("No object detected")
    return x_initial, y_initial, z_initial

def execute_pick_and_place():
    print("Starting pick-and-place sequence")
    ultrasonic_z = read_ultrasonic()
    print(f"Ultrasonic Z height: {ultrasonic_z} cm")
    z_final = -25  # Floor where items will be picked
    x_final, y_final = 150, 100
    detect_object_position_from_camera()
    print("Queuing movement commands...")

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    index = dType.SetEndEffectorGripper(api, enableCtrl=1, on=1, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    time.sleep(2)

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_initial, y_initial, z_initial + 30, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    index = dType.SetEndEffectorGripper(api, enableCtrl=1, on=0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    time.sleep(2)

    index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_final, y_final, z_final + 30, 0, isQueued=1)[1]
    while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
        time.sleep(0.01)

    print("Commands sent. Pick-and-place should be executing now.")
    dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=1)  # Set gripper to rest
    

# === Manual Control GUI (Remove for autonomous version) ===
def start_gui():
    def on_run():
        threading.Thread(target=execute_pick_and_place).start()

    def show_camera():
        def update():
            cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not cap.isOpened():
                print("Failed to open camera")
                return

            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_red = (0, 120, 70)
                upper_red = (10, 255, 255)
                mask = cv2.inRange(hsv, lower_red, upper_red)
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        c = max(contours, key=cv2.contourArea)
                        x, y, w, h = cv2.boundingRect(c)
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Draw origin and quadrant lines
                cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 1)  # vertical center line
                cv2.line(frame, (0, 240), (640, 240), (255, 0, 0), 1)  # horizontal center line
                cv2.circle(frame, (320, 240), 5, (255, 0, 255), -1)     # center point

                # Quadrant labels
                cv2.putText(frame, "I", (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(frame, "II", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(frame, "III", (100, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(frame, "IV", (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

                cv2.imshow("Dobot Camera Feed", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()

        threading.Thread(target=update).start()


    root = tk.Tk()
    root.title("Dobot Manual Control")
    tk.Button(root, text="KILL Program", command=lambda: (dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=0), dType.SetQueuedCmdStopExec(api), dType.SetHOMECmd(api, temp=1, isQueued=0), dType.DisconnectDobot(api), root.quit(), exit()), bg='red', fg='white', height=2, width=25).pack(pady=10)
    tk.Button(root, text="Run Pick and Place", command=on_run, height=2, width=25).pack(padx=20, pady=20)
    tk.Button(root, text="Show Camera Feed", command=show_camera, height=2, width=25).pack(pady=10)
    tk.Button(root, text="Exit", command=lambda: (root.destroy(), dType.DisconnectDobot(api))).pack(pady=10)

    def track_and_grab():
        def wait_for_cmd(result):
            print(f"Command result: {result}")
            if result and len(result) > 1:
                idx = result[1]
                while dType.GetQueuedCmdCurrentIndex(api)[0] < idx:
                    time.sleep(0.01)
            else:
                print("Failed to queue command.")

        def worker():
            cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            time.sleep(0.5)  # Allow camera auto-exposure to settle
            for _ in range(5):  # Discard first few unstable frames
                cap.read()
            ret, frame = cap.read()
            cap.release()
            if not ret:
                print("Failed to capture image")
                return

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = (0, 120, 70)
            upper_red = (10, 255, 255)
            mask = cv2.inRange(hsv, lower_red, upper_red)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                print(f"Tracking object at pixel: ({cx}, {cy})")

                # Removed pixel-to-mm multipliers
                frame_height = 480
                frame_center_x, frame_center_y = 320, 240  # Origin at center

                dx_px = cx - frame_center_x
                dy_px = frame_center_y - cy

                # Direct offset without multiplier
                dx_px = cx - 320  # Relative to center of 640px frame
                dy_px = 240 - cy  # Relative to center of 480px frame

                if cx >= 320 and cy <= 240:  # Quadrant I
                    dx_mm = abs(dx_px) * 0.25
                    dy_mm = abs(dy_px) * 0.25
                elif cx < 320 and cy <= 240:  # Quadrant II
                    dx_mm = -abs(dx_px) * 0.25
                    dy_mm = abs(dy_px) * 0.25
                elif cx < 320 and cy > 240:  # Quadrant III
                    dx_mm = -abs(dx_px) * 0.25
                    dy_mm = -abs(dy_px) * 0.25
                else:  # Quadrant IV
                    dx_mm = abs(dx_px) * 0.25
                    dy_mm = -abs(dy_px) * 0.25

                x_robot = x_initial + dx_mm
                y_robot = y_initial + dy_mm
                z_robot = -25  # Floor level for grabbing
                print(f"Moving to: ({x_robot}, {y_robot}, {z_robot})")

                wait_for_cmd(dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_robot, y_robot, z_robot + 30, 0, isQueued=1))
                wait_for_cmd(dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_robot, y_robot, z_robot, 0, isQueued=1))
                wait_for_cmd(dType.SetEndEffectorGripper(api, enableCtrl=1, on=1, isQueued=1))
                time.sleep(1)
                wait_for_cmd(dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_robot, y_robot, z_robot + 30, 0, isQueued=1))
                dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=1)
            else:
                print("No object detected")

        threading.Thread(target=worker).start()

    tk.Button(root, text="Track and Grab", command=track_and_grab, height=2, width=25).pack(pady=10)
    root.mainloop()

print("Starting GUI...")
start_gui()
