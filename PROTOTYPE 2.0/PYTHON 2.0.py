import tkinter as tk
import serial
import threading
import time
from collections import deque
import random
# Configuration Constants
arduino_PORT = 'COM9'
arduino_BAUD_RATE = 115200


RS232_PORT = 'COM10'  # COM port for RS232 motor control
RS232_BAUD_RATE = 38400  # Baud rate for RS232 motor control
RS232_TIMEOUT = 1  # Timeout for RS232 communication
motor_running = False  # Flag to indicate whether the motor is running
motor_interval = 1.5  # Interval in seconds for motor movements
motor_move_count = 0  # Counter to keep track of motor movements
last_frequency_update_time = time.time()  # Timestamp of the last frequency update
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
motor_serial = connect_serial(RS232_PORT, RS232_BAUD_RATE, RS232_TIMEOUT)
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
                    laser_distance = float(data[0])
                    ultra_distance = int(data[1])
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
    if sensor_type == "ultrasonic":
        if distance > 10:
            status = "Hogging!"
        elif distance < 10:
            status = "Sagging!"
    elif sensor_type == "laser":
        if distance > 100:
            status = "Hogging!"
        elif distance < 100:
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
        laser_range_label_value.config(text=f"{range_value} mm/s")
        laser_previous_value_label_value.config(text=f"{previous_value} ")
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
root = tk.Tk()
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1000x600")

# Main container for frames
sensor = tk.Frame(root)
sensor.pack(pady=10)

# Create GUI Labels for Ultrasonic
# Create the container frame for sensors
main_container = tk.LabelFrame(sensor, text="Sensors", padx=10, pady=10)
main_container.pack(side="left", padx=10)

# Ultrasonic sensor frame
ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=10)  # Use grid layout for better control

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
laser_frame.grid(row=0, column=1, padx=10)  # Place laser frame next to ultrasonic sensor frame

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

gyro_label_value = tk.Label(gyro_frame, text="0 mm", font=("Arial", 20))
gyro_label_value.grid(row=0, column=0, pady=5)


# Add buttons below both the ultrasonic and laser sensor frames but still within the main container
button_frame = tk.Frame(main_container)
button_frame.grid(row=1, column=0, columnspan=3, pady=10)  # Place buttons below the frames

start_button = tk.Button(button_frame, text="Start Measuring", command=start_measuring, font=("Arial", 16))
start_button.grid(row=0, column=0, padx=10)

stop_button = tk.Button(button_frame, text="Stop Measuring", command=stop_measuring, font=("Arial", 16))
stop_button.grid(row=0, column=2, padx=10)


# Motor control frame
motor = tk.Frame(root)
motor.pack(pady=20)

motor_container = tk.LabelFrame(motor, text="Motor Control", padx=10, pady=10)
motor_container.pack(side="left", padx=10)

frequency_label_value = tk.Label(motor_container, text="0 moves/min", font=("Arial", 20))
frequency_label_value.pack(pady=10)

start_motor_button = tk.Button(motor_container, text="Start Motor", command=start_motor, font=("Arial", 16))
start_motor_button.pack(side="left", padx=10)

stop_motor_button = tk.Button(motor_container, text="Stop Motor", command=stop_motor, font=("Arial", 16))
stop_motor_button.pack(side="left", padx=10)



root.mainloop()
5
