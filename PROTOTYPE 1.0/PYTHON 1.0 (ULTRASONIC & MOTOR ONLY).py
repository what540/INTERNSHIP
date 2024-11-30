import tkinter as tk
import serial
import threading
import time
from collections import deque
import random

# Configuration Constants
arduino_PORT = 'COM9'  # COM port for Arduino
arduino_BAUD_RATE = 115200  # Baud rate for Arduino communication
RS232_PORT = 'COM10'  # COM port for RS232 motor control
RS232_BAUD_RATE = 38400  # Baud rate for RS232 motor control
RS232_TIMEOUT = 1  # Timeout for RS232 communication

# Ultrasonic Sensor Variables
measuring = False  # Flag to indicate whether ultrasonic sensor is measuring
distance_values = deque()  # Queue to store recent distance values
previous_distance = None  # Stores the previous distance value for range calculations
last_range_update_time = time.time()  # Timestamp of the last range update

# Motor Control Variables
motor_running = False  # Flag to indicate whether the motor is running
motor_interval = 1.5  # Interval in seconds for motor movements
motor_move_count = 0  # Counter to keep track of motor movements
last_frequency_update_time = time.time()  # Timestamp of the last frequency update

# Serial Connections
def connect_serial(port, baud_rate, timeout=None):
    """
    Connect to a serial port with specified parameters.
    Returns the serial connection object if successful, else returns None.
    """
    try:
        return serial.Serial(port, baud_rate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None

# Establish serial connections
arduinoData = connect_serial(arduino_PORT, arduino_BAUD_RATE)
motor_serial = connect_serial(RS232_PORT, RS232_BAUD_RATE, RS232_TIMEOUT)
time.sleep(1)  # Delay to ensure connections are established

# Ultrasonic Sensor Functions
def read_distance():
    """
    Continuously read distance values from the ultrasonic sensor and update the GUI.
    """
    global measuring, previous_distance, last_range_update_time
    while True:
        if arduinoData.in_waiting > 0:  # Check if there's data in the input buffer
            try:
                # Read and decode data from the Arduino, then split it by commas
                data = arduinoData.readline().decode('utf-8').strip().split(",")
                if len(data) == 3:  # Ensure there are 3 values in the data
                    ultra_distance = int(data[0])  # Get the ultrasonic distance value
                    laser_distance = float(data[0])  # Laser distance (not used here)
                    gyro = float(data[0])  # Gyro value (not used here)
                    
                    # Update the distance label in the GUI
                    root.after(0, update_distance_label, ultra_distance)
                    distance_values.append(ultra_distance)
                    
                    # Calculate range and update the status
                    current_time = time.time()
                    if previous_distance is not None:
                        status = "OK!!"  # Default status
                        if ultra_distance > 10:
                            status = "Hogging!"  # Triggered if the distance is greater than 10
                        elif ultra_distance < 10:
                            status = "Sagging!"  # Triggered if the distance is less than 10
                        root.after(0, update_hogging_status, status)
                    
                    # Update range every second
                    if current_time - last_range_update_time >= 1:
                        root.after(0, calculate_and_update_range, previous_distance, ultra_distance)
                        last_range_update_time = current_time
                        distance_values.clear()
                    
                    previous_distance = ultra_distance
            except Exception as e:
                print(f"Error reading distance: {e}")

def calculate_and_update_range(previous_value, current_value):
    """
    Calculate the range difference between the previous and current distance values.
    Update the corresponding GUI labels with the calculated range.
    """
    range_value = current_value - previous_value if previous_value is not None else 0
    range_label_value.config(text=f"{range_value} cm/s")  # Update the range in cm/s
    previous_value_label_value.config(text=f"{previous_value} cm")  # Previous distance in cm
    current_value_label_value.config(text=f"{current_value} cm")  # Current distance in cm

def update_distance_label(distance):
    """
    Update the distance label in the GUI with the current distance value.
    """
    distance_label_value.config(text=f"{distance} cm")

def update_hogging_status(status):
    """
    Update the status label in the GUI with the current hogging status.
    """
    hogging_label_value.config(text=status)

def start_measuring():
    """
    Start the ultrasonic distance measurement by starting a new thread.
    """
    global measuring
    if not measuring:
        measuring = True
        threading.Thread(target=read_distance, daemon=True).start()

def stop_measuring():
    """
    Stop the ultrasonic distance measurement.
    """
    global measuring
    measuring = False

# Motor Control Functions
def send_ascii_command(ser, command):
    """
    Send an ASCII command to the motor via serial communication.
    """
    try:
        ser.write((command + '\r').encode())  # Send command to the motor
        time.sleep(0.1)  # Short delay for the motor to process the command
        if ser.in_waiting > 0:  # Check if there's a response
            response = ser.read(ser.in_waiting).decode('utf-8').strip()
            print(f"Motor Response: {response}")
    except Exception as e:
        print(f"Error sending motor command: {e}")

def move_motor_to_position():
    """
    Move the motor to a random position at specified intervals.
    This function runs in a separate thread.
    """
    global motor_move_count, motor_running
    while motor_running and motor_serial:
        try:
            # Generate random position and speed for the motor
            position = random.randint(29000, 41000)
            speed = random.randint(50, 250)
            print(f"Moving motor to position {position} at speed {speed}...")
            send_ascii_command(motor_serial, f"S.1={speed}")  # Set motor speed
            send_ascii_command(motor_serial, f"P.1={position}")  # Move motor to position
            send_ascii_command(motor_serial, "^.1")  # Start motor movement
            motor_move_count += 1  # Increment move count
            time.sleep(motor_interval)  # Wait before moving again
        except Exception as e:
            print(f"Motor Error: {e}")
            break

def start_motor():
    """
    Start the motor control by starting a new thread for motor movement.
    """
    global motor_running, last_frequency_update_time, motor_move_count
    if not motor_running:
        motor_running = True
        motor_move_count = 0  # Reset move count
        last_frequency_update_time = time.time()  # Reset frequency update time
        threading.Thread(target=move_motor_to_position, daemon=True).start()  # Start motor movement thread
        threading.Thread(target=update_motor_frequency, daemon=True).start()  # Start frequency update thread

def stop_motor():
    """
    Stop the motor control by setting the motor_running flag to False.
    """
    global motor_running
    motor_running = False

def update_motor_frequency():
    """
    Update the motor movement frequency (moves per minute) every 60 seconds.
    """
    global motor_move_count, last_frequency_update_time
    while motor_running:
        current_time = time.time()
        if current_time - last_frequency_update_time >= 60:  # Update every minute
            frequency = motor_move_count  # Number of moves in the last minute
            root.after(0, update_frequency_label, frequency)  # Update frequency label in GUI
            motor_move_count = 0  # Reset move count
            last_frequency_update_time = current_time  # Reset timestamp

def update_frequency_label(frequency):
    """
    Update the frequency label in the GUI with the current motor frequency.
    """
    frequency_label_value.config(text=f"{frequency:.2f} moves/min")

# GUI Setup
root = tk.Tk()  # Initialize Tkinter root window
root.title("Ultrasonic Sensor & Motor Control")  # Set window title
root.geometry("1000x1000")  # Set window size

# Create GUI Labels and Buttons
frame = tk.Frame(root)
frame.pack(pady=20)

# Labels and values for distance, range, motor frequency, and status
label_column = tk.Frame(frame)
label_column.pack(side="left", padx=20)

distance_label = tk.Label(label_column, text="Distance: ", font=("Arial", 20))
distance_label.pack(pady=10)

range_label = tk.Label(label_column, text="Range: ", font=("Arial", 20))
range_label.pack(pady=10)

frequency_label = tk.Label(label_column, text="Frequency: ", font=("Arial", 20))
frequency_label.pack(pady=10)

hogging_label = tk.Label(label_column, text="Status: ", font=("Arial", 20))
hogging_label.pack(pady=10)

previous_value_label = tk.Label(label_column, text="Previous Value: ", font=("Arial", 20))
previous_value_label.pack(pady=10)

current_value_label = tk.Label(label_column, text="Current Value: ", font=("Arial", 20))
current_value_label.pack(pady=10)

# Column for displaying current values
value_column = tk.Frame(frame)
value_column.pack(side="right", padx=20)

# Displaying initial values for distance, range, motor frequency, and status
distance_label_value = tk.Label(value_column, text="0 cm", font=("Arial", 20))
distance_label_value.pack(pady=10)

range_label_value = tk.Label(value_column, text="0 cm/s", font=("Arial", 20))
range_label_value.pack(pady=10)

frequency_label_value = tk.Label(value_column, text="0 moves/min", font=("Arial", 20))
frequency_label_value.pack(pady=10)

hogging_label_value = tk.Label(value_column, text="OK!!", font=("Arial", 20))
hogging_label_value.pack(pady=10)

previous_value_label_value = tk.Label(value_column, text="0 cm", font=("Arial", 20))
previous_value_label_value.pack(pady=10)

current_value_label_value = tk.Label(value_column, text="0 cm", font=("Arial", 20))
current_value_label_value.pack(pady=10)

# Buttons to control motor and ultrasonic sensor
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

start_button = tk.Button(button_frame, text="Start Ultrasonic Measurement", command=start_measuring, font=("Arial", 20))
start_button.pack(side="left", padx=10)

stop_button = tk.Button(button_frame, text="Stop Ultrasonic Measurement", command=stop_measuring, font=("Arial", 20))
stop_button.pack(side="left", padx=10)

start_motor_button = tk.Button(button_frame, text="Start Motor", command=start_motor, font=("Arial", 20))
start_motor_button.pack(side="left", padx=10)

stop_motor_button = tk.Button(button_frame, text="Stop Motor", command=stop_motor, font=("Arial", 20))
stop_motor_button.pack(side="left", padx=10)

root.mainloop()  # Start the Tkinter main loop to display the GUI
