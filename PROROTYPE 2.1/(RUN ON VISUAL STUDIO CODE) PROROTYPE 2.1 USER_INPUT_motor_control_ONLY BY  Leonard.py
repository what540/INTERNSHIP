import serial
import time
import random
 
# Configuration for the RS232C interface
SERIAL_PORT = 'COM10'  # Replace with your RS232C port
BAUD_RATE = 38400      # Match the motor's baud rate
TIMEOUT = 1            # Communication timeout in seconds
 
def connect_to_rs232():
    """Establish a connection to the RS232C serial port."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to RS232C on {SERIAL_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to RS232C port {SERIAL_PORT}: {e}")
        return None
 
def send_ascii_command(ser, command):
    """Send an ASCII command to the motor and handle the response."""
    try:
        ser.write((command + '\r').encode())  # Ensure commands end with '\r'
        print(f"Command sent: {command}")
        time.sleep(0.1)  # Allow the motor time to process
 
        # Read and print response, if available
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8').strip()
            print(f"Response: {response}")
        else:
            print("No response received.")
    except Exception as e:
        print(f"Error sending command: {e}")
 
def move_motor_to_position(ser, position, speed):
    """
    Send movement commands to the motor.
    Args:
        ser: Serial connection.
        position: Target position (e.g., 1000).
        speed: Movement speed (e.g., 100).
    """
    try:
        # Set speed
        send_ascii_command(ser, f"S.1={speed}")
        # Set target position
        send_ascii_command(ser, f"P.1={position}")
        # Command motor to execute the move
        send_ascii_command(ser, "^.1")
    except Exception as e:
        print(f"Error moving motor: {e}")
 
def main():
    """Main function to execute motor control tasks."""
    ser = connect_to_rs232()
    if not ser:
        return
 
    try:
        # Get user input for movements per 30 seconds
        movements_per_30_seconds = int(input("Enter the number of movements per 30 seconds (e.g., 20): ").strip())
        interval = 30 / movements_per_30_seconds  # Calculate time interval in seconds
 
        print(f"Motor will move {movements_per_30_seconds} times per 30 seconds (~{interval:.2f} seconds per move).")
        print("Type 'exit' at any time to quit.")
 
        while True:
            # Generate random position and speed
            position = random.randint(0, 20000)  # Adjust range based on motor limits
            speed = random.randint(50, 500)     # Random speed
 
            print(f"Moving motor to position {position} with speed {speed}...")
            move_motor_to_position(ser, position, speed)
 
            # Wait for the calculated interval
            time.sleep(interval)
 
            # Check for user exit
            user_input = input("Type 'exit' to quit or press Enter to continue: ").strip().lower()
            if user_input == 'exit':
                print("Exiting motor control.")
                break
 
    finally:
        ser.close()
        print("RS232C connection closed.")
 
if __name__ == "__main__":
    main()