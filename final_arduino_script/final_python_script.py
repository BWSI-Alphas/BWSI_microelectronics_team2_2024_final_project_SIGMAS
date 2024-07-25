import serial
import json

# Configuration
SERIAL_PORT = 'COM3' #change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

def read_serial_data(port, baud_rate, timeout):
    try:
        # Open the serial port
        with serial.Serial(port, baud_rate, timeout=timeout) as ser:
            print(f"Connected to {port} at {baud_rate} baud")

            while True:
                if ser.in_waiting > 0:  # Check if there's any data in the buffer
                    data = ser.readline().decode('utf-8').strip()  # Read a line of data
                    # Decode JSON data
                    json_data = json.loads(data)
                    print("Received JSON:", json_data) # Print received JSON data
                    json_data["led"] = "off"
                    # heres where u would modify the json data
                    # # Modify JSON data
                    if(json_data["person"] == "detected"):
                        if(json_data["status"] == "AUTHORIZED"):
                            json_data["led"] = "on"
                    
                    
                    # Encode JSON data to string
                    json_string = json.dumps(json_data)
                    
                    # Send modified JSON string back to Arduino
                    ser.write((json_string + "\n").encode('utf-8'))
                    print("Sent JSON:", json_string)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")

if __name__ == "__main__":
    read_serial_data(SERIAL_PORT, BAUD_RATE, TIMEOUT)


