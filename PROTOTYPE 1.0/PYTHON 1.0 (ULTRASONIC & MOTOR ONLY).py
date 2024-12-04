import tkinter as tk
import serial
import threading
import time
from collections import deque
import random

# Configuration Constants
ULTRASONIC_PORT = 'COM9'
ULTRASONIC_BAUD_RATE = 9600
RS232_PORT = 'COM10'
RS232_BAUD_RATE = 38400
RS232_TIMEOUT = 1

# Ultrasonic Sensor Variables
measuring = False
distance_values = deque()
previous_distance = None
last_range_update_time = time.time()

# Motor Control Variables
motor_running = False
motor_interval = 1.5  # Default interval for motor movements
motor_move_count = 0
last_frequency_update_time = time.time()


# Serial Connections
def connect_serial(port, baud_rate, timeout=None):
    """Connect to a serial port."""
    try:
        return serial.Serial(port, baud_rate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None


ultrasonic_arduino = connect_serial(ULTRASONIC_PORT, ULTRASONIC_BAUD_RATE)
motor_serial = connect_serial(RS232_PORT, RS232_BAUD_RATE, RS232_TIMEOUT)


# Ultrasonic Sensor Functions
def read_distance():
    """Continuously read distance values from the ultrasonic sensor."""
    global measuring, previous_distance, last_range_update_time

    while measuring and ultrasonic_arduino:
        if ultrasonic_arduino.in_waiting > 0:
            try:
                data = ultrasonic_arduino.readline().decode('utf-8').strip()
                if data.isdigit():
                    distance = int(data)
                    root.after(0, update_distance_label, distance)
                    distance_values.append(distance)

                    current_time = time.time()
                    if previous_distance is not None:
                        status = "OK!!"
                        if distance > 10:
                            status = "Hogging!"
                        elif distance < 10:
                            status = "Sagging!"
                        root.after(0, update_hogging_status, status)

                    if current_time - last_range_update_time >= 1:
                        root.after(0, calculate_and_update_range, previous_distance, distance)
                        last_range_update_time = current_time
                        distance_values.clear()

                    previous_distance = distance
            except Exception as e:
                print(f"Error reading distance: {e}")


def calculate_and_update_range(previous_value, current_value):
    """Calculate and update the range difference between previous and current values."""
    range_value = current_value - previous_value if previous_value is not None else 0
    range_label_value.config(text=f"{range_value} cm/s")
    previous_value_label_value.config(text=f"{previous_value} cm")
    current_value_label_value.config(text=f"{current_value} cm")


def update_distance_label(distance):
    distance_label_value.config(text=f"{distance} cm")


def update_hogging_status(status):
    hogging_label_value.config(text=status)


def start_measuring():
    """Start ultrasonic distance measurement."""
    global measuring
    if not measuring:
        measuring = True
        threading.Thread(target=read_distance, daemon=True).start()


def stop_measuring():
    """Stop ultrasonic distance measurement."""
    global measuring
    measuring = False


# Motor Control Functions
def send_ascii_command(ser, command):
    """Send ASCII command to the motor via serial connection."""
    try:
        ser.write((command + '\r').encode())
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8').strip()
            print(f"Motor Response: {response}")
    except Exception as e:
        print(f"Error sending motor command: {e}")


def move_motor_to_position():
    """Move the motor to random positions at specified intervals."""
    global motor_move_count, motor_running

    while motor_running and motor_serial:
        try:
            position = random.randint(29000, 41000)
            speed = random.randint(50, 250)
            print(f"Moving motor to position {position} at speed {speed}...")
            send_ascii_command(motor_serial, f"S.1={speed}")
            send_ascii_command(motor_serial, f"P.1={position}")
            send_ascii_command(motor_serial, "^.1")
            motor_move_count += 1
            time.sleep(motor_interval)
        except Exception as e:
            print(f"Motor Error: {e}")
            break


def start_motor():
    """Start motor control."""
    global motor_running, last_frequency_update_time, motor_move_count
    if not motor_running:
        motor_running = True
        motor_move_count = 0
        last_frequency_update_time = time.time()
        threading.Thread(target=move_motor_to_position, daemon=True).start()
        threading.Thread(target=update_motor_frequency, daemon=True).start()


def stop_motor():
    """Stop motor control."""
    global motor_running
    motor_running = False


def update_motor_frequency():
    """Update motor movement frequency based on a 30-second window."""
    global motor_move_count, last_frequency_update_time

    while motor_running:
        current_time = time.time()
        if current_time - last_frequency_update_time >= 60:
            frequency = motor_move_count  # Moves per minute
            root.after(0, update_frequency_label, frequency)
            motor_move_count = 0
            last_frequency_update_time = current_time


def update_frequency_label(frequency):
    frequency_label_value.config(text=f"{frequency:.2f} moves/min")


# GUI Setup
root = tk.Tk()
root.title("Ultrasonic Sensor & Motor Control")
root.geometry("600x600")

# Create GUI Labels and Buttons
frame = tk.Frame(root)
frame.pack(pady=20)

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

value_column = tk.Frame(frame)
value_column.pack(side="right", padx=20)

distance_label_value = tk.Label(value_column, text="0 cm", font=("Arial", 20))
distance_label_value.pack(pady=10)

range_label_value = tk.Label(value_column, text="0 cm/s", font=("Arial", 20))
range_label_value.pack(pady=10)

frequency_label_value = tk.Label(value_column, text="0.00 moves/min", font=("Arial", 20))
frequency_label_value.pack(pady=10)

hogging_label_value = tk.Label(value_column, text="OK", font=("Arial", 20))
hogging_label_value.pack(pady=10)

previous_value_label_value = tk.Label(value_column, text="N/A", font=("Arial", 20))
previous_value_label_value.pack(pady=10)

current_value_label_value = tk.Label(value_column, text="N/A", font=("Arial", 20))
current_value_label_value.pack(pady=10)

# Buttons for Ultrasonic and Motor Control
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

start_ultrasonic_button = tk.Button(button_frame, text="Start Measuring", command=start_measuring, font=("Arial", 16))
start_ultrasonic_button.pack(side="left", padx=10)

stop_ultrasonic_button = tk.Button(button_frame, text="Stop Measuring", command=stop_measuring, font=("Arial", 16))
stop_ultrasonic_button.pack(side="left", padx=10)

start_motor_button = tk.Button(button_frame, text="Start Motor", command=start_motor, font=("Arial", 16))
start_motor_button.pack(side="left", padx=10)

stop_motor_button = tk.Button(button_frame, text="Stop Motor", command=stop_motor, font=("Arial", 16))
stop_motor_button.pack(side="left", padx=10)

root.mainloop()
