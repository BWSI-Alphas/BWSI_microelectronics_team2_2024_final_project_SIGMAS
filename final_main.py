import cv2
import serial
import time
import sys
import numpy as np
import json
import threading
from FaceRecognition2 import FaceRecognition
from BodyTracker import process_frame
import serial

port = "COM5"
ser = serial.Serial(port=port, baudrate=9600, timeout=.1)

camera_on = False
servo_x = 90
servo_y = 90

def receive_json_from_arduino():
    if ser.in_waiting > 0:
        json_data = ser.readline().decode().strip()
        try:
            data = json.loads(json_data)
            camera_on = data.get('camera_on', False)
            servo_x = data.get('servo_x', 90)
            servo_y = data.get('servo_y', 90)
            print(f"Received from Arduino: {data}")
            return camera_on, servo_x, servo_y
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
            return None, None, None

def send_json(camera_on, servo_x, servo_y):
    data = {
        "camera_on": camera_on,
        "servo_x": servo_x,
        "servo_y": servo_y
    }
    json_data = json.dumps(data)
    ser.write(json_data.encode())
    print(f"Sent to Arduino: {json_data}")

def send_servo_command(servo, angle):
    try:
        data = {"servo": servo, "angle": angle}
        command = json.dumps(data) + '\n'
        ser.write(command.encode())
        time.sleep(0.05)  # Delay to prevent buffer overflow
    except Exception as e:
        print(f"Error sending servo command: {e}")

def control_servos(center_x, center_y, ws, hs):
    try:
        # Invert X axis to align with expected servo movement
        servoX = np.interp(center_x, [0, ws], [180, 0])
        servoY = np.interp(center_y, [0, hs], [0, 180])
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))
        send_servo_command("x", int(servoX))
        send_servo_command("y", int(servoY))
        print(f"Servo X: {int(servoX)} deg, Servo Y: {int(servoY)} deg")
    except Exception as e:
        print(f"Error controlling servos: {e}")

def video_capture_thread(cap, frame_queue):
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_queue.append(frame)
        if len(frame_queue) > 1:  # Keep only the latest frame to reduce lag
            frame_queue.pop(0)

try:
    camera_on = False
    face_ver = False
    servo_x = 90
    servo_y = 90

    currentDir = r'C:\Users\Noah Lee\OneDrive\Documents\GitHub\face_detection\faces'

    fr = FaceRecognition(currentDir)
    URL = "http://192.168.1.121:81/stream"
    cap = cv2.VideoCapture(0)

    ws, hs = 320, 240

    cap.set(3, ws)
    cap.set(4, hs)

    if not cap.isOpened():
        sys.exit('Video source not found')

    frame_queue = []

    # Start the video capture thread
    capture_thread = threading.Thread(target=video_capture_thread, args=(cap, frame_queue))
    capture_thread.start()

    frame_counter = 0
    
    time.sleep(2)

    while True:
        while camera_on:
            if frame_queue:
                frame = frame_queue.pop(0)

                center_x, center_y = process_frame(frame, ws, hs)
                if center_x is not None and center_y is not None:
                    control_servos(center_x, center_y, ws, hs)
                else:
                    control_servos(90, 90, ws, hs)

                recognized, annotated_frame = fr.process_frame(frame)
                frame = annotated_frame

                cv2.imshow('Face Recognition', frame)

            if cv2.waitKey(1) == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        capture_thread.join()  # Ensure the capture thread is cleaned up

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)  # Exit the script with error status
