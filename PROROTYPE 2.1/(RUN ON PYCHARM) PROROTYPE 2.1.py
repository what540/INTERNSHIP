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
sensor_values = {"ultrasonic": deque(maxlen=2), "laser": deque(maxlen=2), "gyro": deque()}
previous_values = {"ultrasonic": None, "laser": None}
last_update_times = {"ultrasonic": time.time(), "laser": time.time(), "gyro": time.time()}

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

    while measuring and arduinoData:
        if arduinoData.in_waiting > 0:
            try:
                data = arduinoData.readline().decode('utf-8').strip().split(",")
                if len(data) == 3:
                    laser_distance = int(data[0])
                    ultra_distance = int(data[1])
                    gyro = float(data[2])
                    # Update Ultrasonic Sensor Data
                    update_sensor_data("ultrasonic", ultra_distance)

                    # Update Laser Sensor Data
                    update_sensor_data("laser", laser_distance)

                    update_gyro_label(gyro)  # Update the gyro value separately


            except Exception as e:
                print(f"Error reading sensor data: {e}")

def update_gyro_label(gyro):
    """Update the gyro label with the current gyro value."""
    gyro_label_value.config(text=f"{gyro} deg/s")


def update_sensor_data(sensor_type, distance):
    # Update range difference every second
    current_time = time.time()
    """Update sensor data and GUI labels."""
    global previous_values, last_update_times

    root.after(0, update_distance_label, distance, sensor_type)
    sensor_values[sensor_type].append(distance)

    status = "OK!"

    # Check for hogging/sagging status; used as 10cm original displacement between two object for reference
    if sensor_type == "ultrasonic":
        if distance > 10:
            status = "Hogging!"
        elif distance < 10:
            status = "Sagging!"
        elif distance == 10:
            status = "OK!"

    elif sensor_type == "laser":
        if distance > 100:
            status = "Hogging!"
        elif distance < 100:
            status = "Sagging!"
        elif distance == 100:
            status = "OK!"

    root.after(0, update_hogging_status, status, sensor_type)


    if current_time - last_update_times[sensor_type] >= 1: #after 1s
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
        laser_range_label_value.config(text=f"{range_value} mm/s")
        laser_previous_value_label_value.config(text=f"{previous_value} mm")
        laser_current_value_label_value.config(text=f"{current_value} mm")

def update_distance_label(distance, sensor_type):
    if sensor_type == "ultrasonic":
        ultrasonic_distance_label_value.config(text=f"{distance} cm")
    elif sensor_type == "laser":
        laser_distance_label_value.config(text=f"{distance} mm")

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
root.geometry("1200x700")

# Main container for frames
sensor = tk.Frame(root)
sensor.pack(pady=10)

# Create GUI Labels for Ultrasonic
# Create the container frame for sensors
main_container = tk.LabelFrame(sensor, text="Sensors", padx=30, pady=30)
main_container.pack(side="left", padx=10)

# Ultrasonic sensor frame
ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=30)  # Use grid layout for better control

ultrasonic_distance_label_value = tk.Label(ultrasonic_frame, text="0 cm", font=("Arial", 20))
ultrasonic_distance_label_value.grid(row=0, column=0, pady=5)

ultrasonic_range_label_value = tk.Label(ultrasonic_frame, text="0 cm/s", font=("Arial", 20))
ultrasonic_range_label_value.grid(row=1, column=0, pady=5)

ultrasonic_hogging_label_value = tk.Label(ultrasonic_frame, text="OK", font=("Arial", 20))
ultrasonic_hogging_label_value.grid(row=2, column=0, pady=5)

ultrasonic_previous_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 20))
ultrasonic_previous_value_label_value.grid(row=3, column=0, pady=5)

ultrasonic_current_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 20))
ultrasonic_current_value_label_value.grid(row=4, column=0, pady=5)

# Laser sensor frame
laser_frame = tk.LabelFrame(main_container, text="Laser Sensor", padx=10, pady=10)
laser_frame.grid(row=0, column=1, padx=30)  # Place laser frame next to ultrasonic sensor frame

laser_distance_label_value = tk.Label(laser_frame, text="0 mm", font=("Arial", 20))
laser_distance_label_value.grid(row=0, column=0, pady=5)

laser_range_label_value = tk.Label(laser_frame, text="0 mm/s", font=("Arial", 20))
laser_range_label_value.grid(row=1, column=0, pady=5)

laser_hogging_label_value = tk.Label(laser_frame, text="OK", font=("Arial", 20))
laser_hogging_label_value.grid(row=2, column=0, pady=5)

laser_previous_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 20))
laser_previous_value_label_value.grid(row=3, column=0, pady=5)

laser_current_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 20))
laser_current_value_label_value.grid(row=4, column=0, pady=5)

gyro_frame = tk.LabelFrame(main_container, text="Gyro Sensor", padx=10, pady=10)
gyro_frame.grid(row=0, column=2, padx=10)

gyro_label_value = tk.Label(gyro_frame, text="0.0 deg/s", font=("Arial", 20))
gyro_label_value.grid(row=0, column=0, pady=5)


# Add buttons below both the ultrasonic and laser sensor frames but still within the main container
button_frame = tk.Frame(main_container)
button_frame.grid(row=1, column=0, columnspan=3, pady=10)  # Place buttons below the frames

start_button = tk.Button(button_frame, text="Start Measuring", command=start_measuring, font=("Arial", 16))
start_button.grid(row=0, column=0, padx=10)

stop_button = tk.Button(button_frame, text="Stop Measuring", command=stop_measuring, font=("Arial", 16))
stop_button.grid(row=0, column=2, padx=10)

root.mainloop()
