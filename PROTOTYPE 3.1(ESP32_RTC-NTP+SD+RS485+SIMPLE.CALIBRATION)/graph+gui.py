import tkinter as tk
import serial
import threading
import serial.tools.list_ports
from tkinter import ttk
from time import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np


# Configuration Constants
arduino_BAUD_RATE = 115200
connection = False
# Sensor Variables
measuring = False

ultra_distance_array =[]
laser_distance_array =[]
time_string = ""
time_string_array=[]

# Serial Connections
def get_available_ports():
    """Get all available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Initialize arduinoData as None
arduinoData = None


def on_select():
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
    root.after(5000, update_gui_status)

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
    root.after(0, update_stop_measurement_ui)

def read_sensor_data():
    global measuring, arduinoData, connection,time_string
    if connection and arduinoData:
        try:
            arduinoData.reset_input_buffer() #https://stackoverflow.com/questions/60766714/pyserial-flush-vs-reset-input-buffer-reset-output-buffer
            while measuring:
                if arduinoData.in_waiting > 0:
                    data = arduinoData.readline().decode('utf-8').strip().split(",")    #by Paul McWhorter 25 Jan 2022 https://www.youtube.com/watch?v=VN3HJm3spRE&ab_channel=PaulMcWhorter
                    if len(data) ==7:
                        ultra_distance = float(data[0])
                        laser_distance = float(data[1])
                        range_ultra_distance = float(data[2])
                        range_laser_distance = float(data[3])
                        prev_ultra_distance = float(data[4])
                        prev_laser_distance = float(data[5])
                        gyro = float(data[6])

                        # Update Ultrasonic Sensor Data
                        root.after(0, ultrasonic_update_distance_label, ultra_distance)
                        root.after(0, laser_update_distance_label, laser_distance)
                        root.after(0, ultrasonic_calculate_and_update_range, prev_ultra_distance, range_ultra_distance)
                        root.after(0, laser_calculate_and_update_range, prev_laser_distance, range_laser_distance)

                        ultrasonic_update_sensor_data(ultra_distance)
                        laser_update_sensor_data(laser_distance)

                        laser_distance_rounded = round(laser_distance, 2)
                        ultra_distance_rounded = round(ultra_distance, 2)

                        ultra_distance_array.append(ultra_distance_rounded)
                        laser_distance_array.append(laser_distance_rounded)
                        time_string_array.append(time_string)


                        # Update Gyro Sensor Data
                        update_gyro_label(gyro)  # Update the gyro value separately
        except serial.SerialException as e:
            print(f"Error with serial connection: {e}")
            connection = False
        except Exception as e:
            print(f"Error reading sensor data: {e}")



def update_gyro_label(gyro):
    gyro_label_value.config(text=f"{gyro} deg/s")

def ultrasonic_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 10:
        status = "Hogging!"
    elif distance < 10:
        status = "Sagging!"
    elif distance == 10:
        status = "OK!"
    root.after(0, ultrasonic_update_hogging_status, status)


def laser_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 100:
        status = "Hogging!"
    elif distance < 100:
        status = "Sagging!"
    elif distance == 100:
        status = "OK!"
    root.after(0, laser_update_hogging_status, status)

def ultrasonic_update_distance_label(distance):
    """Update distance label on GUI based on sensor type."""
    ultrasonic_distance_label_value.config(text=f"{distance} mm")

def laser_update_distance_label(distance):
    """Update distance label on GUI based on sensor type."""
    laser_distance_label_value.config(text=f"{distance} mm")

def ultrasonic_update_hogging_status(status):
    """Update the hogging status on the GUI based on sensor type."""
    ultrasonic_hogging_label_value.config(text=status)

def laser_update_hogging_status(status):
    """Update the hogging status on the GUI based on sensor type."""
    laser_hogging_label_value.config(text=status)

def ultrasonic_calculate_and_update_range(previous_value, range_value):
    """Calculate and update the range difference."""
    ultrasonic_range_label_value.config(text=f"{range_value} mm/s")
    ultrasonic_previous_value_label_value.config(text=f"{previous_value} mm")

def laser_calculate_and_update_range(previous_value, range_value):
    """Calculate and update the range difference."""
    laser_range_label_value.config(text=f"{range_value} mm/s")
    laser_previous_value_label_value.config(text=f"{previous_value} mm")

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
            ultrasonic_distance_label_value.config(text="0 mm")
            ultrasonic_range_label_value.config(text="0 mm/s")
            ultrasonic_hogging_label_value.config(text="OK")
            ultrasonic_previous_value_label_value.config(text="N/A")
            laser_distance_label_value.config(text="0 mm")
            laser_range_label_value.config(text="0 mm/s")
            laser_hogging_label_value.config(text="OK")
            laser_previous_value_label_value.config(text="N/A")
            gyro_label_value.config(text="0.0 deg/s")


def clock_update():
    global time_string_array, time_string
    time_string = strftime("%H:%M:%S") #by Bro Code 16 Sept 2020  https://www.youtube.com/watch?v=l7IMBy4_nhA&t=183s&ab_channel=BroCode
    time_label.config(text=time_string)

    date_string = strftime("%a, %d %b %Y")
    date_label.config(text=date_string)
    root.after(1000,clock_update)

def plot_graph_update():
    global ultra_distance_array, laser_distance_array, time_string_array, time_string

    ax.clear()  # Clear the axes
    ax.plot(time_string_array, ultra_distance_array, marker='x', markersize=5, label='Ultrasonic Distance')
    ax.plot(time_string_array, laser_distance_array, marker='o', markersize=5, label='Laser Distance')

    if len(ultra_distance_array) > 10:  # if array size >10, pop the first index of all 3 arrays https://www.programiz.com/python-programming/methods/list/pop
        ultra_distance_array.pop(0)
        laser_distance_array.pop(0)
        time_string_array.pop(0)

    ax.set_title("Distance over Time")
    ax.set_xlabel("Time (HH:MM:SS)")
    ax.set_ylabel("Distance (mm)")
    ax.legend()
    canvas.draw()
    root.after(50, plot_graph_update)

# GUI Setup
root = tk.Tk() #by Bro Code 22 Sept 2020 https://www.youtube.com/watch?v=lyoyTlltFVU&ab_channel=BroCode
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1200x780")

# Main container for frame
date_label = tk.Label(root,font=("Arial",10,"bold"))
date_label.pack()

time_label = tk.Label(root,font=("Arial",10))
time_label.pack()

# ComboBox for port selection
port_combobox = ttk.Combobox(root, values=get_available_ports()) #Chatgpt
port_combobox.pack(pady=3)

# Button to select port and connect
select_button = ttk.Button(root, text="Select Port", command=lambda: on_select())
select_button.pack(pady=3)

refresh_button = ttk.Button(root, text="Refresh Ports", command=refresh_ports)
refresh_button.pack(pady=3)

close_button = ttk.Button(root, text="Close Connection", command=close_serial, state=tk.DISABLED)
close_button.pack(pady=3)

# Connection status label
status_label = ttk.Label(root, text="Connection: Not Established", foreground="red")
status_label.pack(pady=3)

# Start/Stop buttons
start_button = ttk.Button(root, text="Start Measuring", command=start_measuring, state=tk.DISABLED)
start_button.pack(pady=3)

stop_button = ttk.Button(root, text="Stop Measuring", command=stop_measuring, state=tk.DISABLED)
stop_button.pack(pady=3)
sensor = tk.Frame(root)
sensor.pack(pady=10)

main_container = tk.LabelFrame(sensor, text="Sensors", padx=20, pady=20)
main_container.pack(side="left", padx=10)

ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=30)  # Use grid layout for better control

ultrasonic_distance_label = tk.Label(ultrasonic_frame, text="Current Distance (mm):", font=("Arial", 12))
ultrasonic_distance_label.grid(row=0, column=0, pady=5, sticky="w")
ultrasonic_distance_label_value = tk.Label(ultrasonic_frame, text="0 mm", font=("Arial", 12))
ultrasonic_distance_label_value.grid(row=0, column=1, pady=5)

ultrasonic_previous_value_label = tk.Label(ultrasonic_frame, text="Previous:", font=("Arial", 12))
ultrasonic_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
ultrasonic_previous_value_label_value = tk.Label(ultrasonic_frame, text="N/A", font=("Arial", 12))
ultrasonic_previous_value_label_value.grid(row=1, column=1, pady=5)

ultrasonic_range_label = tk.Label(ultrasonic_frame, text="Range (mm/s):", font=("Arial", 12))
ultrasonic_range_label.grid(row=2, column=0, pady=5, sticky="w")
ultrasonic_range_label_value = tk.Label(ultrasonic_frame, text="0 mm/s", font=("Arial", 12))
ultrasonic_range_label_value.grid(row=2, column=1, pady=5)

ultrasonic_hogging_label = tk.Label(ultrasonic_frame, text="Status:", font=("Arial", 12))
ultrasonic_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
ultrasonic_hogging_label_value = tk.Label(ultrasonic_frame, text="OK", font=("Arial", 12))
ultrasonic_hogging_label_value.grid(row=3, column=1, pady=5)

# Laser sensor frame
laser_frame = tk.LabelFrame(main_container, text="Laser Sensor", padx=10, pady=10)
laser_frame.grid(row=0, column=1, padx=30)  # Place laser frame next to ultrasonic sensor frame

laser_distance_label = tk.Label(laser_frame, text="Current Distance (mm):", font=("Arial", 12))
laser_distance_label.grid(row=0, column=0, pady=5, sticky="w")
laser_distance_label_value = tk.Label(laser_frame, text="0 mm", font=("Arial", 12))
laser_distance_label_value.grid(row=0, column=1, pady=5)

laser_previous_value_label = tk.Label(laser_frame, text="Previous:", font=("Arial", 12))
laser_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
laser_previous_value_label_value = tk.Label(laser_frame, text="N/A", font=("Arial", 12))
laser_previous_value_label_value.grid(row=1, column=1, pady=5)

laser_range_label = tk.Label(laser_frame, text="Range (mm/s):", font=("Arial", 12))
laser_range_label.grid(row=2, column=0, pady=5, sticky="w")
laser_range_label_value = tk.Label(laser_frame, text="0 mm/s", font=("Arial", 12))
laser_range_label_value.grid(row=2, column=1, pady=5)

laser_hogging_label = tk.Label(laser_frame, text="Status:", font=("Arial", 12))
laser_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
laser_hogging_label_value = tk.Label(laser_frame, text="OK", font=("Arial", 12))
laser_hogging_label_value.grid(row=3, column=1, pady=5)

gyro_frame = tk.LabelFrame(main_container, text="Gyro Sensor", padx=10, pady=10)
gyro_frame.grid(row=0, column=2, padx=10)

gyro_label = tk.Label(gyro_frame, text="Gyro X (deg/s):", font=("Arial", 12))
gyro_label.grid(row=0, column=0, pady=5, sticky="w")
gyro_label_value = tk.Label(gyro_frame, text="0.0 deg/s", font=("Arial", 12))
gyro_label_value.grid(row=0, column=1, pady=5)

fig, ax = plt.subplots() #CodersLegacy  21 Feb 2023 https://youtu.be/lVTC8CvScQo?si=TdkXePkDE8nejzmu
graphFrame = tk.Frame(root)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
canvas.draw()


clock_update()

plot_graph_update()
# Start the Tkinter event loop
root.mainloop()