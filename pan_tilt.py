import cv2
import numpy as np
import json
import serial
import time

# Initialize serial connection
ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)  # Adjust port as needed
time.sleep(2)  # Allow time for the serial connection to establish

def send_json(camera_on, servo_x, servo_y):
    try:
        data = {
            "camera_on": camera_on,
            "servo_x": servo_x,
            "servo_y": servo_y
        }
        json_data = json.dumps(data) + '\n'  # Ensure newline character for proper serial transmission
        ser.write(json_data.encode())
        print(f"Sent to Arduino: {json_data}")
        time.sleep(0.05)  # Delay to prevent buffer overflow
    except serial.SerialException as e:
        print(f"Error sending JSON to Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Initialize the video capture
URL = "http://192.168.1.121:81/stream"  # Change stream URL as needed
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200)
time.sleep(2)

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

FRAME_H, FRAME_W = frame.shape[:2]
CENTER_X = int(FRAME_W / 2 + 0.5)
CENTER_Y = int(FRAME_H / 2 + 0.5)

clock = time.clock


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error getting image")
        continue

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    # Process faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        face_x = x + w // 2
        face_y = y + h // 2

        turn_x = float(face_x -CENTER_X)
        turn_y = float(face_y - CENTER_Y)

        turn_x /= float(CENTER_X)
        turn_y /= float(CENTER_Y)

        # Increase the scaling factor
        scale_factor = 5  # Adjust factor as needed
        turn_x *= scale_factor
        turn_y *= scale_factor

        cam_pan = - turn_x
        cam_tilt =  turn_y

        # Clamp to 0-180 degrees
        print(cam_pan-90, cam_tilt-90)
        cam_pan = max(0, min(180, cam_pan))
        cam_tilt = max(0, min(180, cam_tilt))

        # Send updated servo positions
        send_json(True, int(cam_pan), int(cam_tilt))

        break

    # Display the video captured
    cv2.imshow('Video', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and destroy windows
cap.release()
cv2.destroyAllWindows()
