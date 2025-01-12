import tkinter as tk
import serial
import threading
import time
from collections import deque
import serial.tools.list_ports
from tkinter import ttk

# Configuration Constants
arduino_BAUD_RATE = 115200
connection = False
# Sensor Variables
measuring = False
sensor_values = {"ultrasonic": deque(maxlen=2), "laser": deque(maxlen=2), "gyro": deque()}
previous_values = {"ultrasonic": None, "laser": None}
last_update_times = {"ultrasonic": time.time(), "laser": time.time(), "gyro": time.time()}


# Serial Connections
def get_available_ports():
    """Get all available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Initialize arduinoData as None
arduinoData = None


def on_select(event):
    """Event handler for port selection."""
    selected_port = port_combobox.get()  # Get the selected port from combobox
    print(f"User selected: {selected_port}")

    # Connect to the selected port
    global arduinoData
    arduinoData = serial.Serial(selected_port, arduino_BAUD_RATE)

    # Start by setting the status to "Establishing"
    status_label.config(text="Connection: Establishing", foreground="orange")
    select_button.config(state=tk.DISABLED)
    refresh_button.config(state=tk.DISABLED)

    # update the status to "Established"
    root.after(1000, update_gui_status)  # 15 seconds

def refresh_ports():
    """Refresh the list of available serial ports and update the combobox."""
    available_ports = get_available_ports()  # Get updated list of ports
    port_combobox['values'] = available_ports  # Update the combobox with new ports



def update_gui_status():
    """Update the GUI to reflect the connection status."""
    global connection
    if arduinoData:
        status_label.config(text="Connection: Established", foreground="green")
        select_button.config(state=tk.DISABLED)
        refresh_button.config(state=tk.DISABLED)
        start_button.config(state=tk.NORMAL)
        close_button.config(state=tk.NORMAL)
        connection = True
    else:
        status_label.config(text="Connection: Failed", foreground="red")


def start_measuring():
    """Start measuring for both sensors."""
    global measuring, arduinoData
    if not measuring:
        measuring = True
        threading.Thread(target=read_sensor_data, daemon=True).start()
    root.after(0, update_start_measurement_ui)


def stop_measuring():
    """Stop measuring for both sensors."""
    global measuring, arduinoData
    measuring = False
    ultrasonic_distance_label_value.config(text="0 cm")
    ultrasonic_range_label_value.config(text="0 cm/s")
    ultrasonic_hogging_label_value.config(text="OK")
    ultrasonic_previous_value_label_value.config(text="N/A")
    laser_distance_label_value.config(text="0 mm")
    laser_range_label_value.config(text="0 mm/s")
    laser_hogging_label_value.config(text="OK")
    laser_previous_value_label_value.config(text="N/A")
    gyro_label_value.config(text="0.0 deg/s")
    root.after(0, update_stop_measurement_ui)


def read_sensor_data():
    """Read data from the Arduino and update GUI labels."""
    global measuring, previous_values, last_update_times, arduinoData, connection
    if connection and arduinoData:
        try:
            while measuring:
                if arduinoData.in_waiting > 0:
                    data = arduinoData.readline().decode('utf-8').strip().split(",")
                    if len(data) == 3:
                        laser_distance = int(data[0])
                        ultra_distance = int(data[1])
                        gyro = float(data[2])

                        # Update Ultrasonic Sensor Data
                        update_sensor_data("ultrasonic", ultra_distance)

                        # Update Laser Sensor Data
                        update_sensor_data("laser", laser_distance)

                        # Update Gyro Sensor Data
                        update_gyro_label(gyro)  # Update the gyro value separately
        except serial.SerialException as e:
            print(f"Error with serial connection: {e}")
            connection = False
        except Exception as e:
            print(f"Error reading sensor data: {e}")



def update_gyro_label(gyro):
    """Update the gyro label with the current gyro value."""
    gyro_label_value.config(text=f"{gyro} deg/s")


def update_sensor_data(sensor_type, distance):
    """Update sensor data and GUI labels."""
    global previous_values, last_update_times

    current_time = time.time()

    # Update range difference every second
    root.after(0, update_distance_label, distance, sensor_type)
    sensor_values[sensor_type].append(distance)

    status = "OK!"

    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
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

    if current_time - last_update_times[sensor_type] >= 1:  # after 1s
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
    elif sensor_type == "laser":
        laser_range_label_value.config(text=f"{range_value} mm/s")
        laser_previous_value_label_value.config(text=f"{previous_value} mm")


def update_distance_label(distance, sensor_type):
    """Update distance label on GUI based on sensor type."""
    if sensor_type == "ultrasonic":
        ultrasonic_distance_label_value.config(text=f"{distance} cm")
    elif sensor_type == "laser":
        laser_distance_label_value.config(text=f"{distance} mm")


def update_hogging_status(status, sensor_type):
    """Update the hogging status on the GUI based on sensor type."""
    if sensor_type == "ultrasonic":
        ultrasonic_hogging_label_value.config(text=status)
    elif sensor_type == "laser":
        laser_hogging_label_value.config(text=status)


def update_start_measurement_ui():
    """Instantly update UI when starting measurement."""
    status_label.config(text="Measuring...", foreground="green")
    start_button.config(state=tk.DISABLED)
    stop_button.config(state=tk.NORMAL)
    close_button.config(state=tk.DISABLED)


def update_stop_measurement_ui():
    """Update UI when stopping measurement."""
    status_label.config(text="Stop", foreground="red")
    start_button.config(state=tk.NORMAL)
    stop_button.config(state=tk.DISABLED)
    close_button.config(state=tk.NORMAL)

def close_serial():
    """Close the serial connection."""
    global arduinoData, connection
    if arduinoData:
        try:
            if arduinoData.is_open:
                arduinoData.close()
                print("Serial connection closed.")
        except Exception as e:
            print(f"Error closing serial connection: {e}")
        finally:
            arduinoData = None
            connection = False
            status_label.config(text="Connection: Not Established", foreground="red")
            select_button.config(state=tk.NORMAL)  # Re-enable the port selection button
            refresh_button.config(state=tk.NORMAL)
            close_button.config(state=tk.DISABLED)
            start_button.config(state=tk.DISABLED)
            stop_button.config(state=tk.DISABLED)
            ultrasonic_distance_label_value.config(text="0 cm")
            ultrasonic_range_label_value.config(text="0 cm/s")
            ultrasonic_hogging_label_value.config(text="OK")
            ultrasonic_previous_value_label_value.config(text="N/A")
            laser_distance_label_value.config(text="0 mm")
            laser_range_label_value.config(text="0 mm/s")
            laser_hogging_label_value.config(text="OK")
            laser_previous_value_label_value.config(text="N/A")
            gyro_label_value.config(text="0.0 deg/s")



# GUI Setup
root = tk.Tk()
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1200x700")

# Main container for frames


# ComboBox for port selection
port_combobox = ttk.Combobox(root, values=get_available_ports())
port_combobox.pack(pady=20)

# Button to select port and connect
select_button = ttk.Button(root, text="Select Port", command=lambda: on_select(None))
select_button.pack(pady=5)

refresh_button = ttk.Button(root, text="Refresh Ports", command=refresh_ports)
refresh_button.pack(pady=5)

close_button = ttk.Button(root, text="Close Connection", command=close_serial, state=tk.DISABLED)
close_button.pack(pady=5)

# Connection status label
status_label = ttk.Label(root, text="Connection: Not Established", foreground="red")
status_label.pack(pady=10)

# Start/Stop buttons
start_button = ttk.Button(root, text="Start Measuring", command=start_measuring, state=tk.DISABLED)
start_button.pack(pady=10)

stop_button = ttk.Button(root, text="Stop Measuring", command=stop_measuring, state=tk.DISABLED)
stop_button.pack(pady=10)
sensor = tk.Frame(root)
sensor.pack(pady=10)

main_container = tk.LabelFrame(sensor, text="Sensors", padx=30, pady=30)
main_container.pack(side="left", padx=10)

ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=30)  # Use grid layout for better control

ultrasonic_distance_label = tk.Label(ultrasonic_frame, text="Current Distance (cm):", font=("Arial", 12))
ultrasonic_distance_label.grid(row=0, column=0, pady=5, sticky="w")
ultrasonic_distance_label_value = tk.Label(ultrasonic_frame, text="0 cm", font=("Arial", 20))
ultrasonic_distance_label_value.grid(row=0, column=1, pady=5)

ultrasonic_previous_value_label = tk.Label(ultrasonic_frame, text="Previous:", font=("Arial", 12))
ultrasonic_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
ultrasonic_previous_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 20))
ultrasonic_previous_value_label_value.grid(row=1, column=1, pady=5)

ultrasonic_range_label = tk.Label(ultrasonic_frame, text="Range (cm/s):", font=("Arial", 12))
ultrasonic_range_label.grid(row=2, column=0, pady=5, sticky="w")
ultrasonic_range_label_value = tk.Label(ultrasonic_frame, text="0 cm/s", font=("Arial", 20))
ultrasonic_range_label_value.grid(row=2, column=1, pady=5)

ultrasonic_hogging_label = tk.Label(ultrasonic_frame, text="Status:", font=("Arial", 12))
ultrasonic_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
ultrasonic_hogging_label_value = tk.Label(ultrasonic_frame, text="OK", font=("Arial", 20))
ultrasonic_hogging_label_value.grid(row=3, column=1, pady=5)

# Laser sensor frame
laser_frame = tk.LabelFrame(main_container, text="Laser Sensor", padx=10, pady=10)
laser_frame.grid(row=0, column=1, padx=30)  # Place laser frame next to ultrasonic sensor frame

laser_distance_label = tk.Label(laser_frame, text="Current Distance (mm):", font=("Arial", 12))
laser_distance_label.grid(row=0, column=0, pady=5, sticky="w")
laser_distance_label_value = tk.Label(laser_frame, text="0 mm", font=("Arial", 20))
laser_distance_label_value.grid(row=0, column=1, pady=5)

laser_previous_value_label = tk.Label(laser_frame, text="Previous:", font=("Arial", 12))
laser_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
laser_previous_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 20))
laser_previous_value_label_value.grid(row=1, column=1, pady=5)

laser_range_label = tk.Label(laser_frame, text="Range (mm/s):", font=("Arial", 12))
laser_range_label.grid(row=2, column=0, pady=5, sticky="w")
laser_range_label_value = tk.Label(laser_frame, text="0 mm/s", font=("Arial", 20))
laser_range_label_value.grid(row=2, column=1, pady=5)

laser_hogging_label = tk.Label(laser_frame, text="Status:", font=("Arial", 12))
laser_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
laser_hogging_label_value = tk.Label(laser_frame, text="OK", font=("Arial", 20))
laser_hogging_label_value.grid(row=3, column=1, pady=5)


gyro_frame = tk.LabelFrame(main_container, text="Gyro Sensor", padx=10, pady=10)
gyro_frame.grid(row=0, column=2, padx=10)

gyro_label = tk.Label(gyro_frame, text="Gyro X (deg/s):", font=("Arial", 12))
gyro_label.grid(row=0, column=0, pady=5, sticky="w")
gyro_label_value = tk.Label(gyro_frame, text="0.0 deg/s", font=("Arial", 20))
gyro_label_value.grid(row=0, column=1, pady=5)


# Start the Tkinter event loop
root.mainloop()
