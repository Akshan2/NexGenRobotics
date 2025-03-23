import time
import cv2
import threading
import tkinter as tk
from tkinter import simpledialog, messagebox
import numpy as np
import json
import os
import DobotDllType as dType
# import pyzed.sl as sl  # ZED camera commented out

# ---------------------------
# Persistent Calibration File
# ---------------------------
CALIB_FILE = "calibration_data.json"

# ---------------------------
# Dobot Initialization
# ---------------------------
api = dType.load()
state = dType.ConnectDobot(api, '', 115200)[0]
if state != dType.DobotConnect.DobotConnect_NoError:
    print("Failed to connect to Dobot")
    exit()
print("Connected to Dobot")

dType.SetQueuedCmdClear(api)
dType.SetHOMEParams(api, 200, 0, 50, 0)
homeCmdIndex = dType.SetHOMECmd(api, temp=1, isQueued=1)[1]
dType.SetQueuedCmdStartExec(api)

# Wait for homing to complete
index = dType.SetHOMECmd(api, temp=1, isQueued=1)[1]
while dType.GetQueuedCmdCurrentIndex(api)[0] < index:
    time.sleep(0.1)
print("Homing started...")
while True:
    currentIndex = dType.GetQueuedCmdCurrentIndex(api)[0]
    if currentIndex >= homeCmdIndex:
        break
    time.sleep(0.1)
print("Homing complete.")

dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100)
dType.SetPTPCoordinateParams(api, 100, 100, 100, 100)
dType.SetPTPJumpParams(api, 10, 50)
print("All home done")

# ---------------------------
# Global Settings & Variables
# ---------------------------
x_initial, y_initial, z_initial = 200, 0, 50

TRIG_PORT = 15
ECHO_PORT = 16
dType.SetIOMultiplexing(api, TRIG_PORT, 0, isQueued=0)
dType.SetIOMultiplexing(api, ECHO_PORT, 0, isQueued=0)

# Movement multiplier (for potential future use)
move_multiplier = 0.003  # initial value

# Calibration transformation parameters (offset applied AFTER scaling):
# Transformation: robot_x = scale_x * pixel_x + offset_x
calib_scale_x = 1.0
calib_offset_x = 0.0
calib_scale_y = 1.0
calib_offset_y = 0.0

# List to hold calibration pairs: each item is ((px, py), (robot_x, robot_y))
calibration_data = []

# ---------------------------
# Persistent Calibration Storage Functions
# ---------------------------
def load_calibration():
    global calibration_data, calib_scale_x, calib_offset_x, calib_scale_y, calib_offset_y
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, 'r') as f:
                data = json.load(f)
                calibration_data = data.get("calibration_data", [])
                calib_scale_x = data.get("calib_scale_x", 1.0)
                calib_offset_x = data.get("calib_offset_x", 0.0)
                calib_scale_y = data.get("calib_scale_y", 1.0)
                calib_offset_y = data.get("calib_offset_y", 0.0)
            print("Loaded calibration data from disk.")
        except Exception as e:
            print("Failed to load calibration data:", e)
    else:
        print("No calibration file found. Starting fresh.")

def save_calibration():
    data = {
        "calibration_data": calibration_data,
        "calib_scale_x": calib_scale_x,
        "calib_offset_x": calib_offset_x,
        "calib_scale_y": calib_scale_y,
        "calib_offset_y": calib_offset_y
    }
    try:
        with open(CALIB_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        print("Calibration data saved to disk.")
    except Exception as e:
        print("Failed to save calibration data:", e)

load_calibration()

# ---------------------------
# Helper Functions
# ---------------------------
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
            return 7.94
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
    except Exception as e:
        print("Ultrasonic read failed. Using default height.", e)
        return 20.0

def detect_object_position(frame):
    # Apply a Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy
    else:
        return None

def wait_for_cmd(cmd_result):
    if cmd_result and len(cmd_result) > 1:
        idx = cmd_result[1]
        while dType.GetQueuedCmdCurrentIndex(api)[0] < idx:
            time.sleep(0.01)
    else:
        print("Failed to queue command.")

def take_snapshot():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ret, frame = cap.read()
    cap.release()
    if ret:
        return frame
    else:
        print("Failed to capture snapshot.")
        return None

def get_average_detection(num_snapshots=3, delay=0.2):
    centers = []
    for i in range(num_snapshots):
        frame = take_snapshot()
        if frame is not None:
            detection = detect_object_position(frame)
            print(f"Snapshot {i+1} detection: {detection}")
            if detection is not None:
                centers.append(detection)
        time.sleep(delay)
    if centers:
        avg_x = sum(pt[0] for pt in centers) / len(centers)
        avg_y = sum(pt[1] for pt in centers) / len(centers)
        avg_detection = (int(avg_x), int(avg_y))
        print(f"Average detection from {num_snapshots} snapshots: {avg_detection}")
        return avg_detection
    else:
        return None

def compute_calibration():
    global calib_scale_x, calib_offset_x, calib_scale_y, calib_offset_y
    if len(calibration_data) < 2:
        print("Not enough calibration data. Need at least 2 points per axis.")
        return False
    A = []
    bx = []
    by = []
    for (px, py), (rx, ry) in calibration_data:
        A.append([px, 1])
        bx.append(rx)
        by.append(ry)
    A = np.array(A)
    bx = np.array(bx)
    by = np.array(by)
    sol_x, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
    sol_y, _, _, _ = np.linalg.lstsq(A, by, rcond=None)
    calib_scale_x, calib_offset_x = sol_x
    calib_scale_y, calib_offset_y = sol_y
    print(f"Calibration computed:")
    print(f"  X: scale = {calib_scale_x:.3f}, offset = {calib_offset_x:.3f}")
    print(f"  Y: scale = {calib_scale_y:.3f}, offset = {calib_offset_y:.3f}")
    save_calibration()
    return True

def pixel_to_robot(px, py):
    return calib_scale_x * px + calib_offset_x, calib_scale_y * py + calib_offset_y

# ---------------------------
# Pick-and-Place Routine
# ---------------------------
def execute_pick_and_place():
    print("Starting pick-and-place sequence")
    ultrasonic_z = read_ultrasonic()
    print(f"Ultrasonic Z height: {ultrasonic_z} cm")
    z_final = -25
    x_final, y_final = 150, 100
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_initial, y_initial, z_initial + 30, 0, isQueued=1)
    wait_for_cmd(cmd)
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_initial, y_initial, z_initial, 0, isQueued=1)
    wait_for_cmd(cmd)
    cmd = dType.SetEndEffectorGripper(api, enableCtrl=1, on=1, isQueued=1)
    wait_for_cmd(cmd)
    time.sleep(2)
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_initial, y_initial, z_initial + 30, 0, isQueued=1)
    wait_for_cmd(cmd)
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_final, y_final, z_final + 30, 0, isQueued=1)
    wait_for_cmd(cmd)
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_final, y_final, z_final, 0, isQueued=1)
    wait_for_cmd(cmd)
    cmd = dType.SetEndEffectorGripper(api, enableCtrl=1, on=0, isQueued=1)
    wait_for_cmd(cmd)
    time.sleep(2)
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x_final, y_final, z_final + 30, 0, isQueued=1)
    wait_for_cmd(cmd)
    print("Pick-and-place sequence executed.")
    dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=1)

# ---------------------------
# Calibration Routine
# ---------------------------
def calibrate():
    global calibration_data
    frame = take_snapshot()
    if frame is None:
        messagebox.showerror("Error", "Failed to capture snapshot for calibration.")
        return
    detection = detect_object_position(frame)
    if detection is None:
        messagebox.showerror("Error", "No object detected in snapshot.")
        return
    px, py = detection
    cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)
    cv2.imshow("Calibration Snapshot", frame)
    cv2.waitKey(500)
    cv2.destroyWindow("Calibration Snapshot")
    rx = simpledialog.askfloat("Calibration", "Enter the robot's X coordinate for the detected target:", minvalue=-1000, maxvalue=1000)
    ry = simpledialog.askfloat("Calibration", "Enter the robot's Y coordinate for the detected target:", minvalue=-1000, maxvalue=1000)
    if rx is None or ry is None:
        messagebox.showerror("Error", "Calibration canceled or invalid input.")
        return
    calibration_data.append(((px, py), (rx, ry)))
    messagebox.showinfo("Calibration", f"Calibration point added.\nTotal points: {len(calibration_data)}")
    if len(calibration_data) >= 2:
        if compute_calibration():
            messagebox.showinfo("Calibration", "Calibration complete and transformation computed.")

# ---------------------------
# Snapshot-based Track and Grab (3 Steps, Centering the Block)
# ---------------------------
def track_and_grab():
    def worker():
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            print("Failed to open camera for snapshot mode.")
            return

        # Use a fixed conversion factor (mm per pixel) for relative correction.
        mm_per_pixel = 0.3
        # Desired center in pixel coordinates (camera center).
        desired_center = (320, 240)
        # Assume the robot's current target position starts at (x_initial, y_initial)
        x_robot, y_robot, z_robot = x_initial, y_initial, -25

        for step in range(3):
            avg_detection = get_average_detection(num_snapshots=3, delay=0.2)
            if avg_detection is None:
                print(f"Step {step+1}: No detection obtained. Retrying.")
                time.sleep(0.5)
                continue
            cx, cy = avg_detection

            # Compute pixel error relative to the desired center.
            error_x = desired_center[0] - cx
            error_y = desired_center[1] - cy
            print(f"Step {step+1}: Detected average pixel = ({cx}, {cy})")
            print(f"Step {step+1}: Pixel error = ({error_x}, {error_y})")

            # Convert pixel error to mm offset.
            delta_x = error_x * mm_per_pixel
            delta_y = error_y * mm_per_pixel

            # Debug: Print current robot pose (assumes dType.GetPose(api) returns (ret, x, y, z, r))
            pose = dType.GetPose(api)
            if pose and len(pose) >= 4:
                print(f"Robot pose before move: x = {pose[1]:.2f}, y = {pose[2]:.2f}, z = {pose[3]:.2f}")

            # Update the target position by adding the computed offset.
            x_robot += delta_x
            y_robot += delta_y

            print(f"Step {step+1}: Moving to new target: x = {x_robot:.2f}, y = {y_robot:.2f}")
            cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                                  x_robot, y_robot, z_robot + 30, 0, isQueued=1)
            wait_for_cmd(cmd)
            time.sleep(1.0)
            pose = dType.GetPose(api)
            if pose and len(pose) >= 4:
                print(f"Robot pose after move: x = {pose[1]:.2f}, y = {pose[2]:.2f}, z = {pose[3]:.2f}")

        cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                              x_robot, y_robot, z_robot, 0, isQueued=1)
        wait_for_cmd(cmd)
        gripper_cmd = dType.SetEndEffectorGripper(api, 1, 1, isQueued=1)
        wait_for_cmd(gripper_cmd)
        time.sleep(1)
        cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                              x_robot, y_robot, z_robot + 30, 0, isQueued=1)
        wait_for_cmd(cmd)
        dType.SetEndEffectorGripper(api, 0, 0, isQueued=1)
        cap.release()
    threading.Thread(target=worker).start()

# ---------------------------
# GUI Setup
# ---------------------------
def update_multiplier(val):
    global move_multiplier
    move_multiplier = float(val)
    print(f"Movement multiplier updated to: {move_multiplier}")

def start_gui():
    root = tk.Tk()
    root.title("Dobot Manual Control")
    
    tk.Label(root, text="Calibration").pack(pady=5)
    tk.Button(root, text="Capture Calibration Point", command=calibrate, height=2, width=25).pack(pady=5)
    
    tk.Label(root, text="Movement Multiplier (unused in snapshot mode)").pack(pady=5)
    multiplier_slider = tk.Scale(root, from_=0.001, to=0.01, resolution=0.001,
                                 orient=tk.HORIZONTAL, command=update_multiplier)
    multiplier_slider.set(move_multiplier)
    multiplier_slider.pack(pady=5)
    
    tk.Button(root, text="KILL Program", command=lambda: (
        dType.SetEndEffectorGripper(api, enableCtrl=0, on=0, isQueued=0),
        dType.SetQueuedCmdStopExec(api),
        dType.SetHOMECmd(api, temp=1, isQueued=0),
        dType.DisconnectDobot(api),
        root.quit(),
        exit()
    ), bg='red', fg='white', height=2, width=25).pack(pady=10)
    
    tk.Button(root, text="Run Pick and Place", command=lambda: threading.Thread(target=execute_pick_and_place).start(),
              height=2, width=25).pack(padx=20, pady=20)
    tk.Button(root, text="Track and Grab (Center Block, 3 Steps)", command=track_and_grab, height=2, width=25).pack(pady=10)
    tk.Button(root, text="Exit", command=lambda: (root.destroy(), dType.DisconnectDobot(api))).pack(pady=10)
    
    root.mainloop()

print("Starting GUI...")
start_gui()
