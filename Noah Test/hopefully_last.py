import serial
import json
import cv2
import time
import threading
from FaceRecognition2 import FaceRecognition
import os

# Global variables
camera = False
servo_pos_x = 90
servo_pos_y = 90

# Configuration
SERIAL_PORT = 'COM4'  # Change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

# Pan servo settings
servo_pan_speed = 0.01
pulse_pan_min = 0
pulse_pan_max = 180

# Tilt servo settings
servo_tilt_speed = 0.01
pulse_tilt_min = 0
pulse_tilt_max = 180

threshold_x = 10  # Num pixels can be from CENTER_X
threshold_y = 10  # Num pixels can be from CENTER_Y

dir_x = -1
dir_y = 1

# Initialize serial communication
def init_serial(port, baud_rate, timeout):
    ser = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
    ser.setDTR(False)  # Drop DTR
    time.sleep(1)  # Wait for a second
    ser.setDTR(True)  # Raise DTR
    return ser

# Thread function for receiving data
def receive_json(arduino):
    global camera
    while True:
        if arduino.in_waiting > 0:
            data = arduino.readline().decode().strip()
            if data:
                try:
                    json_data = json.loads(data)
                    print(f"Received from Arduino: {json_data}")
                    initialize_variables(json_data)
                except json.JSONDecodeError:
                    print("Received invalid JSON data.")
        time.sleep(0.01)  # Reduce CPU usage

# Thread function for sending data
def send_json(arduino, camera_on, servo_x, servo_y):
    try:
        if send_handshake(arduino):
            data = {
                "camera": camera_on,
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

def send_handshake(arduino):
    arduino.write(b'handshake\n')
    response = arduino.readline().decode().strip()
    return response == 'ack'

def initialize_variables(json_data):
    global camera
    camera = json_data.get("camera", True)
    print(f"Camera: {camera}")

def video_processing_loop(arduino):
    global servo_pos_x, servo_pos_y, camera
    currentDir = r'C:\Users\Noah Lee\OneDrive\Documents\GitHub\face_detection\faces'  # Change directory
    fr = FaceRecognition(currentDir)
    cap = cv2.VideoCapture(0)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Variables for FPS calculation
    fps_start_time = time.time()
    frame_count = 0

    while True:
        if camera:
            # Capture the frame
            ret, frame = cap.read()
            if not ret:
                print("Error getting image")
                continue

            # Reduce frame resolution for processing
            frame = cv2.resize(frame, (640, 480))

            recognized, annotated_frame = fr.process_frame(frame)
            frame = annotated_frame
            if recognized:
                camera = False

            FRAME_H, FRAME_W = frame.shape[:2]
            CENTER_X = int(FRAME_W / 2 + 0.5)
            CENTER_Y = int(FRAME_H / 2 + 0.5)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=5)

            largest_face_size = 0
            largest_face_bb = None
            for (x, y, w, h) in faces:
                face_size = w * h
                if face_size > largest_face_size: 
                    largest_face_size = face_size
                    largest_face_bb = (x, y, w, h)
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            if largest_face_bb is not None:
                face_x = largest_face_bb[0] + int((largest_face_bb[2]) / 2 + 0.5)
                face_y = largest_face_bb[1] + int((largest_face_bb[3]) / 2 + 0.5)

                cv2.line(frame, (CENTER_X, CENTER_Y), (face_x, face_y), (0, 255, 0), 2)

                diff_x = face_x - CENTER_X
                if abs(diff_x) <= threshold_x:
                    diff_x = 0

                diff_y = face_y - CENTER_Y
                if abs(diff_y) <= threshold_y:
                    diff_y = 0

                mov_x = dir_x * servo_pan_speed * diff_x
                mov_y = dir_y * servo_tilt_speed * diff_y

                servo_pos_x = servo_pos_x + mov_x
                servo_pos_y = servo_pos_y + mov_y

                servo_pos_x = max(servo_pos_x, pulse_pan_min)
                servo_pos_x = min(servo_pos_x, pulse_pan_max)
                servo_pos_y = max(servo_pos_y, pulse_tilt_min)
                servo_pos_y = min(servo_pos_y, pulse_tilt_max)

                print("Moving to X:", int(servo_pos_x), "Y:", int(servo_pos_y))
                send_json(arduino, camera, int(servo_pos_x), int(servo_pos_y))
            else:
                send_json(arduino, camera, int(servo_pos_x), int(servo_pos_y))

            frame_count += 1
            elapsed_time = time.time() - fps_start_time
            if elapsed_time > 0:
                fps = frame_count / elapsed_time
                fps_text = f"FPS: {fps:.2f}"
            else:
                fps_text = "FPS: N/A"

            cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                camera = False
                break

    send_json(arduino, camera, 90, 90)
    cap.release()
    cv2.destroyAllWindows()

def main():
    global arduino
    arduino = init_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
    time.sleep(2)  # Allow time for initialization

    # Start the threads
    receive_thread = threading.Thread(target=receive_json, args=(arduino,))
    receive_thread.daemon = True
    receive_thread.start()

    video_thread = threading.Thread(target=video_processing_loop, args=(arduino,))
    video_thread.daemon = True
    video_thread.start()

    # Main thread can be used for other tasks or to keep the program running
    while True:
        time.sleep(1)  # Keep the main thread alive

if __name__ == "__main__":
    main()
