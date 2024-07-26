
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
SERIAL_PORT = '/dev/cu.usbmodem142201'  # Change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds
def init_serial(port, baud_rate, timeout):
    ser = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
    ser.setDTR(False)  # Drop DTR
    time.sleep(1)  # Wait for a second
    ser.setDTR(True)  # Raise DTR
    return ser


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
    currentDir = '/Users/joy/Desktop/face_detection/faces'  # Change directory
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

            ret, frame = video_capture.read()
            if not ret:
                break

            recognized, annotated_frame = fr.process_frame(frame)
            frame = annotated_frame
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            if not recognized:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    arduino.write("0".encode())
                    print("Person detected")

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