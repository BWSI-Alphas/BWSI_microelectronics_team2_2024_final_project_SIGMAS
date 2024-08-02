import serial
import os
import json
import cv2
import time
import threading
from queue import Queue
from FaceRecognition2 import FaceRecognition
from twilio.rest import Client

# Global vars
person, status, camera, onceFlag, notFirst, override = False, False, False, False, False, False
serial_data_queue = Queue()

# Your Account SID and Auth Token from twilio.com/console
account_sid = 'AC4bd3f6ed3f3dca017281ff861f8f1ae4'
auth_token = 'a18c0c1b8750eb718fdf447164e770c4'

# Create an instance of the Twilio client
client = Client(account_sid, auth_token)

# Configuration
SERIAL_PORT = '/dev/cu.usbmodem142201'  # Change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

unknowns_dir = "unknowns"
if not os.path.exists(unknowns_dir):
    os.makedirs(unknowns_dir)

def send_handshake():
    arduino.write(b'handshake\n')
    response = arduino.readline().decode().strip()
    return response == 'ack'

# Initialize serial communication
def init_serial(port, baud_rate, timeout):
    try:
        ser = serial.Serial(port=port, baudrate=baud_rate, timeout=timeout)
        ser.setDTR(False)
        time.sleep(1)
        ser.setDTR(True)
        print("Serial connection established")
        return ser
    except serial.SerialException as e:
        print(f"Error initializing serial connection: {e}")
        return None

arduino = init_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
time.sleep(2)  # Allow some time for Arduino to reset

# Send JSON data to Arduino
def send_json(camera_on):
    try:
        data = {
            "camera_on": camera_on,
        }
        json_data = json.dumps(data) + '\n'
        arduino.write(json_data.encode())
        time.sleep(0.05)  # Delay to prevent buffer overflow
    except serial.SerialException as e:
        print(f"Error sending JSON to Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Receive JSON data from Arduino
def receive_json(arduino):
    data_buffer = ""
    try:
        while True:
            if arduino.in_waiting > 0:
                data = arduino.read_until(b'\n').decode()
                data_buffer += data
                if data_buffer.strip():
                    try:
                        json_data = json.loads(data_buffer)
                        serial_data_queue.put(json_data)
                        data_buffer = ""
                    except json.JSONDecodeError:
                        print(f"Received invalid JSON data: {data_buffer}. Continuing to receive...")
                        data_buffer = ""
    except serial.SerialException as e:
        print(f"Error receiving JSON from Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Initialize variables from JSON data
def initialize_variables(json_data):
    print(json_data)
    global person, status, camera
    person = json_data.get("person", False)
    status = json_data.get("status", False)
    camera = json_data.get("camera", False)

# Start a thread for receiving JSON data from Arduino
def start_serial_thread():
    serial_thread = threading.Thread(target=receive_json, args=(arduino,), daemon=True)
    serial_thread.start()

currentDir = "/Users/joy/Desktop/face_detection/faces"  # Change directory
fr = FaceRecognition(currentDir)
time.sleep(2)

URL = "http://192.168.86.40:81/stream"  # Change stream URL as needed
cap = cv2.VideoCapture(URL)
time.sleep(2) 

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variables for FPS calculation
fps_start_time = time.time()
frame_count = 0

# Start the serial communication thread
start_serial_thread()

try:
    while True:
        # Process any new data from the serial thread
        while not serial_data_queue.empty():
            json_data = serial_data_queue.get()
            initialize_variables(json_data)
        
        if status or override:
            print("verified")
            send_json(False)
            # cap.release()
            # cv2.destroyAllWindows()
            break
        
        if camera:
            ret, frame = cap.read()
            if not ret:
                print("Error getting image")
                continue
            FRAME_H, FRAME_W = frame.shape[:2]
            CENTER_X = int(FRAME_W / 2 + 0.5)
            CENTER_Y = int(FRAME_H / 2 + 0.5)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=5)

            recognized, annotated_frame, name = fr.process_frame(frame)
            frame = annotated_frame
            if recognized: 
                camera = False
                override = True
                # message = client.messages.create(
                #     body=f'{name} is authorized and safe to be let in',
                #     from_='+18775062532',  # Your Twilio phone number
                #     to='+13129988056'      # Recipient's phone number | ETHAN WEN's NUMBER :pleading_face:
                # )
                print("Should be ending")
                onceFlag = True
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
                face_x = largest_face_bb[0] + int((largest_face_bb[2]) / 2 + 0.5)
                face_y = largest_face_bb[1] + int((largest_face_bb[3]) / 2 + 0.5)

                # Draw a line from the center to center of face
                cv2.line(frame, (CENTER_X, CENTER_Y), (face_x, face_y), (0, 255, 0), 2)
        

            if not recognized and len(faces) > 0 and not onceFlag:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                unknown_filename = os.path.join(unknowns_dir, f"unknown_{timestamp}.jpg")
                # cv2.imwrite(unknown_filename, frame)
                # print(f"Unrecognized face saved to {unknown_filename}")
                # message = client.messages.create(
                #     body='Intruder Detected!',
                #     from_='+18775062532',  # Your Twilio phone number
                #     to='+13129988056'      # Recipient's phone number | ETHAN WEN's NUMBER :pleading_face:
                # )
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            else:
                onceFlag = False

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
                break
        else:
            if not notFirst:
                print("Waiting for camera to turn on...")
                notFirst = True
        
finally:
    # cap.release()
    # cv2.destroyAllWindows()
    print("Cleaned up and exiting")
