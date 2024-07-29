import serial
import json
import cv2
import time
import sys
import numpy as np
import threading
#from FaceRecognition2 import FaceRecognition
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

def receive_json(arduino):
    data_buffer = ""  # Initialize data_buffer
    try:
        print("Waiting for data from Arduino...")
        while True:
            if arduino.in_waiting > 0:
                data = arduino.read_until(b'\n').decode()
                data_buffer += data
                if data_buffer.strip():  # Check if buffer contains data
                    try:
                        json_data = json.loads(data_buffer)
                        print(f"Received from Arduino: {json_data}")
                        data_buffer = ""  # Clear buffer after successful read
                        initialize_variables(json_data)
                        return json_data
                    except json.JSONDecodeError:
                        print("Received invalid JSON data. Continuing to receive...")
                        data_buffer = ""  # Clear buffer if JSON decoding fails
    except serial.SerialException as e:
        print(f"Error receiving JSON from Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

def initialize_variables(json_data):
    # Initialize variables based on the JSON data
    person = json_data.get("person", False)
    status = json_data.get("status", "UNAUTHORIZED")
    camera = json_data.get("camera", True)
    
    # Print or use these variables as needed
    print(f"Person: {person}")
    print(f"Status: {status}")
    print(f"Camera: {camera}")
        
        
# Initialize serial communication
arduino = init_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
time.sleep(2)

while True:
    receive_json(arduino)