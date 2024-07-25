import serial
import json
import time

# Global variables
camera_on = False
servo_x = 90
servo_y = 90

# Initialize serial connection
ser = serial.Serial('COM5', 9600)  # Replace 'COM3' with the appropriate port for your system
time.sleep(2)  # Wait for the serial connection to initialize

def send_json_to_arduino(camera_on, servo_x, servo_y):
    data = {
        "camera_on": camera_on,
        "servo_x": servo_x,
        "servo_y": servo_y
    }
    json_data = json.dumps(data)
    ser.write(json_data.encode())
    print(f"Sent to Arduino: {json_data}")

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
            return False, 90, 90
    return False, 90, 90

try:
    while True:
        # Send JSON data to Arduino
        camera_on, servo_x, servo_y = receive_json_from_arduino()


        # Wait for a short period before receiving data
        time.sleep(1)

        # Receive JSON data from Arduino
        print(f"Camera On: {camera_on}, Servo X: {servo_x}, Servo Y: {servo_y}")

        # Update global variables for demonstration
        camera_on = not camera_on  # Toggle camera_on for demonstration
        servo_x = (servo_x + 10) % 180  # Update servo_x
        servo_y = (servo_y + 10) % 180  # Update servo_y
        
        send_json_to_arduino(camera_on, servo_x, servo_y)

        
        time.sleep(4)  # Wait before sending the next packet

finally:
    ser.close()
