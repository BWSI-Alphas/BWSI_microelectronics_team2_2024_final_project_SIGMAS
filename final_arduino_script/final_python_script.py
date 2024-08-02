
import serial
import json
import cv2
import time
import sys
import numpy as np
import threading
from FaceRecognition2 import FaceRecognition
import os
import math

# Configuration
SERIAL_PORT = 'COM5'  # Change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

# Pan servo settings
servo_pan_ch = 1  # Adjust as needed
servo_pan_speed = 0.01
pulse_pan_min = 0
pulse_pan_max = 180

# Tilt servo settings
servo_tilt_ch = 1  # Adjust as needed
servo_tilt_speed = 0.01
pulse_tilt_min = 0
pulse_tilt_max = 180

threshold_x = 10  # Num pixels can be from CENTER_X
threshold_y = 10  # Num pixels can be from CENTER_Y

dir_x = -1
dir_y = 1

# Initial servo positions
servo_pos_x = int(((pulse_pan_max - pulse_pan_min) / 2) + pulse_pan_min)
servo_pos_y = int(((pulse_tilt_max - pulse_tilt_min) / 2) + pulse_tilt_min)

def send_handshake():
    arduino.write(b'handshake\n')
    response = arduino.readline().decode().strip()
    return response == 'ack'

def init_serial(port, baud_rate, timeout):
    ser = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
    ser.setDTR(False)  # Drop DTR
    time.sleep(1)  # Wait for a second
    ser.setDTR(True)  # Raise DTR
    return ser

def send_json(camera_on, servo_x, servo_y):
    try:
        if send_handshake():
            data = {
                "camera_on": camera_on,
                "servo_x": servo_x,
                "servo_y": servo_y
            }
            json_data = json.dumps(data) + '\n'  # Ensure newline character for proper serial transmission
            arduino.write(json_data.encode())
            print(f"Sent to Arduino: {json_data}")
            time.sleep(0.05)  # Delay to prevent buffer overflow
        else:
            print("Handshake failed.")
    except serial.SerialException as e:
        print(f"Error sending JSON to Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Initialize serial communication
arduino = init_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
time.sleep(2)

camera_state = False  # This should be a global variable
finish = False

def read_serial_data(port, baud_rate, timeout):
    global camera_state, finish  # Use the global variable
    try:
        with serial.Serial(port, baud_rate, timeout=timeout) as ser:
            print(f"Connected to {port} at {baud_rate} baud")

            while True:
                if ser.in_waiting > 0:  # Check if there's any data in the buffer
                    data = ser.readline().decode('utf-8').strip()  # Read a line of data
                    json_data = json.loads(data)  # Decode JSON data
                    print("Received JSON:", json_data)  # Print received JSON data

                    if json_data.get("camera") == "on":
                        camera_state = True
                    else:
                        camera_state = False  # Update camera state
                    
                    if json_data.get("status") == "AUTHORIZED":
                        finish = True
                    
                    if finish:
                        json_data["led"] = "on"

                    json_string = json.dumps(json_data)  # Encode JSON data to string
                    ser.write((json_string + "\n").encode('utf-8'))  # Send modified JSON string back to Arduino
                    print("Sent JSON:", json_string)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")

def main():
    global camera_state, finish# Use the global variable
    currentDir = r'C:\Users\Noah Lee\OneDrive\Documents\GitHub\face_detection\faces'  # Change directory
    fr = FaceRecognition(currentDir)

    video_capture = None

    serial_thread = threading.Thread(target=read_serial_data, args=(SERIAL_PORT, BAUD_RATE, TIMEOUT))
    serial_thread.start()

    while True:
        if finish:
            pass
        elif camera_state:
            if video_capture is None or not video_capture.isOpened():
                URL = "http://192.168.86.48:81/stream" # port might be different depending on esp32
                video_capture = cv2.VideoCapture(URL)
                if not video_capture.isOpened():
                    print('Video source not found')
                    sys.exit('Video source not found')
                FRAME_H, FRAME_W = frame.shape[:2]
                CENTER_X = FRAME_W // 2
                CENTER_Y = FRAME_H // 2

            ret, frame = video_capture.read()
            if not ret:
                break

            recognized, annotated_frame = fr.process_frame(frame)
            frame = annotated_frame
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            if not recognized:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)


                largest_face_size = 0
                largest_face_bb = None
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    arduino.write("0".encode())
                    print("Person detected")
                    face_size = w * h
                    if face_size > largest_face_size: 
                        largest_face_size = face_size
                        largest_face_bb = (x, y, w, h)
                    
                    # Draw box 
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                
                
                if largest_face_bb is not None: 
                    print("Face : ", largest_face_bb)

                    face_x = largest_face_bb[0] + int((largest_face_bb[2]) / 2 + 0.5)
                    face_y = largest_face_bb[1] + int((largest_face_bb[3]) / 2 + 0.5)

                    # Draw a line from the center to center of face
                    cv2.line(frame, (CENTER_X, CENTER_Y), (face_x, face_y), (0, 255, 0), 2)

                    # Figure out how far away the face is

                    diff_x = face_x - CENTER_X
                    if abs(diff_x)<= threshold_x:
                        diff_x = 0

                    diff_y = face_y - CENTER_Y
                    if abs(diff_y)<= threshold_y:
                        diff_y = 0

                    # Calculate the relative position the servos should move to based on distance
                    mov_x = dir_x * servo_pan_speed * diff_x
                    mov_y = dir_y * servo_tilt_speed * diff_y

                    # Adjust camera position left/right and up/down
                    servo_pos_x = servo_pos_x + mov_x
                    servo_pos_y = servo_pos_y + mov_y

                    # Constrain servo positions to range of servos
                    servo_pos_x = max(servo_pos_x, pulse_pan_min)
                    servo_pos_x = min(servo_pos_x, pulse_pan_max)
                    servo_pos_y = max(servo_pos_y, pulse_pan_min)
                    servo_pos_y = min(servo_pos_y, pulse_pan_max)
                    
                    print("Moving to X:", int(servo_pos_x), "Y:", int(servo_pos_y))

                    send_json(True, int(servo_pos_x), int(servo_pos_y))
                if len(faces) == 0:
                    arduino.write("1".encode())
                    print("No person detected")

                annotated_frame = frame
                cv2.imshow('Face Recognition', annotated_frame)

            if recognized:
                arduino.write("0".encode())
                print("Recognized person detected")
                finish = True
                cv2.imshow('Face Recognition', frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            if video_capture is not None and video_capture.isOpened():
                video_capture.release()
                cv2.destroyAllWindows()
                video_capture = None
            time.sleep(1)

    if video_capture is not None and video_capture.isOpened():
        video_capture.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()