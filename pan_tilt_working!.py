import cv2
import numpy as np
import json
import serial
import time

# Initialize serial connection
ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)  # Replace with your actual port
time.sleep(2)  # Allow time for the serial connection to establish

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
    ser.write(b'handshake\n')
    response = ser.readline().decode().strip()
    return response == 'ack'

def send_json(camera_on, servo_x, servo_y):
    try:
        if send_handshake():
            data = {
                "camera_on": camera_on,
                "servo_x": servo_x,
                "servo_y": servo_y
            }
            json_data = json.dumps(data) + '\n'  # Ensure newline character for proper serial transmission
            ser.write(json_data.encode())
            print(f"Sent to Arduino: {json_data}")
            time.sleep(0.05)  # Delay to prevent buffer overflow
        else:
            print("Handshake failed.")
    except serial.SerialException as e:
        print(f"Error sending JSON to Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Initialize the video capture
URL = "http://192.168.1.121:81/stream"  # Change stream URL as needed
cap = cv2.VideoCapture(URL)
time.sleep(2)

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variables for FPS calculation
fps_start_time = time.time()
frame_count = 0

while True:
    # Capture the frame
    ret, frame = cap.read()

    if not ret:
        print("Error getting image")
        continue

    FRAME_H, FRAME_W = frame.shape[:2]
    CENTER_X = int(FRAME_W / 2 + 0.5)
    CENTER_Y = int(FRAME_H / 2 + 0.5)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=5)

    largest_face_size = 0
    largest_face_bb = None
    for (x, y, w, h) in faces:
        # Find bounding box
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

    # Calculate FPS
    frame_count += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time > 0:
        fps = frame_count / elapsed_time
        fps_text = f"FPS: {fps:.2f}"
    else:
        fps_text = "FPS: N/A"

    # Display FPS on the frame
    cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the video captured
    cv2.imshow('Video', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy windows
cap.release()
cv2.destroyAllWindows()
