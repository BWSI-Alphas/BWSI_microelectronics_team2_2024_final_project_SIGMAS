import serial

# Configuration
SERIAL_PORT = 'COM3'
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
                    ser.write(b"Message Received\n")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")

if __name__ == "__main__":
    read_serial_data(SERIAL_PORT, BAUD_RATE, TIMEOUT)
