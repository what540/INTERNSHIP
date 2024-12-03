import tkinter as tk
import serial
import threading
import time
from collections import deque

# Configuration Constants
arduino_PORT = 'COM9'
arduino_BAUD_RATE = 115200

# Sensor Variables
measuring = False
sensor_values = {"ultrasonic": deque(), "laser": deque()}
previous_values = {"ultrasonic": None, "laser": None}
last_update_times = {"ultrasonic": time.time(), "laser": time.time()}

# Serial Connections
def connect_serial(port, baud_rate, timeout=None):
    """Connect to a serial port."""
    try:
        return serial.Serial(port, baud_rate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None

arduinoData = connect_serial(arduino_PORT, arduino_BAUD_RATE)
time.sleep(1)

# Sensor Functions
def read_sensor_data():
    """Read data from the Arduino and update GUI labels."""
    global measuring, previous_values, last_update_times

    while measuring:
        if arduinoData and arduinoData.in_waiting > 0:
            try:
                data = arduinoData.readline().decode('utf-8').strip().split(",")
                if len(data) == 3:
                    ultra_distance = int(data[0])
                    laser_distance = float(data[1])

                    # Update Ultrasonic Sensor Data
                    update_sensor_data("ultrasonic", ultra_distance)

                    # Update Laser Sensor Data
                    update_sensor_data("laser", laser_distance)

            except Exception as e:
                print(f"Error reading sensor data: {e}")

def update_sensor_data(sensor_type, distance):
    """Update sensor data and GUI labels."""
    global previous_values, last_update_times

    root.after(0, update_distance_label, distance, sensor_type)
    sensor_values[sensor_type].append(distance)

    # Check for hogging/sagging status
    status = "OK!!"
    if distance > 10:
        status = "Hogging!"
    elif distance < 10:
        status = "Sagging!"
    root.after(0, update_hogging_status, status, sensor_type)

    # Update range difference every second
    current_time = time.time()
    if current_time - last_update_times[sensor_type] >= 1:
        previous_value = previous_values[sensor_type]
        root.after(0, calculate_and_update_range, previous_value, distance, sensor_type)
        last_update_times[sensor_type] = current_time
        sensor_values[sensor_type].clear()

    previous_values[sensor_type] = distance

def calculate_and_update_range(previous_value, current_value, sensor_type):
    """Calculate and update the range difference."""
    range_value = current_value - previous_value if previous_value is not None else 0
    if sensor_type == "ultrasonic":
        ultrasonic_range_label_value.config(text=f"{range_value} cm/s")
        ultrasonic_previous_value_label_value.config(text=f"{previous_value} cm")
        ultrasonic_current_value_label_value.config(text=f"{current_value} cm")
    elif sensor_type == "laser":
        laser_range_label_value.config(text=f"{range_value} cm/s")
        laser_previous_value_label_value.config(text=f"{previous_value} cm")
        laser_current_value_label_value.config(text=f"{current_value} cm")

def update_distance_label(distance, sensor_type):
    if sensor_type == "ultrasonic":
        ultrasonic_distance_label_value.config(text=f"{distance} cm")
    elif sensor_type == "laser":
        laser_distance_label_value.config(text=f"{distance} cm")

def update_hogging_status(status, sensor_type):
    if sensor_type == "ultrasonic":
        ultrasonic_hogging_label_value.config(text=status)
    elif sensor_type == "laser":
        laser_hogging_label_value.config(text=status)

def start_measuring():
    """Start measuring for both sensors."""
    global measuring
    if not measuring:
        measuring = True
        threading.Thread(target=read_sensor_data, daemon=True).start()

def stop_measuring():
    """Stop measuring for both sensors."""
    global measuring
    measuring = False

# GUI Setup
root = tk.Tk()
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1000x800")

# Create GUI Labels for Ultrasonic
ultrasonic_frame = tk.LabelFrame(root, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.pack(pady=20)

ultrasonic_distance_label_value = tk.Label(ultrasonic_frame, text="0 cm", font=("Arial", 20))
ultrasonic_distance_label_value.pack()

ultrasonic_range_label_value = tk.Label(ultrasonic_frame, text="0 cm/s", font=("Arial", 20))
ultrasonic_range_label_value.pack()

ultrasonic_hogging_label_value = tk.Label(ultrasonic_frame, text="OK", font=("Arial", 20))
ultrasonic_hogging_label_value.pack()

ultrasonic_previous_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 20))
ultrasonic_previous_value_label_value.pack()

ultrasonic_current_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 20))
ultrasonic_current_value_label_value.pack()

# Create GUI Labels for Laser
laser_frame = tk.LabelFrame(root, text="Laser Sensor", padx=10, pady=10)
laser_frame.pack(pady=20)

laser_distance_label_value = tk.Label(laser_frame, text="0 cm", font=("Arial", 20))
laser_distance_label_value.pack()

laser_range_label_value = tk.Label(laser_frame, text="0 cm/s", font=("Arial", 20))
laser_range_label_value.pack()

laser_hogging_label_value = tk.Label(laser_frame, text="OK", font=("Arial", 20))
laser_hogging_label_value.pack()

laser_previous_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 20))
laser_previous_value_label_value.pack()

laser_current_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 20))
laser_current_value_label_value.pack()

# Buttons for Control
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

start_button = tk.Button(button_frame, text="Start Measuring", command=start_measuring, font=("Arial", 16))
start_button.pack(side="left", padx=10)

stop_button = tk.Button(button_frame, text="Stop Measuring", command=stop_measuring, font=("Arial", 16))
stop_button.pack(side="left", padx=10)

root.mainloop()
