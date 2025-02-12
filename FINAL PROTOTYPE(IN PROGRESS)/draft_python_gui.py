import tkinter as tk

import serial
import threading
import serial.tools.list_ports
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import scipy.stats as st
import numpy as np

from time import strftime

# Configuration Constants
arduino_BAUD_RATE = 115200
connection = False
# Sensor Variables
measuring = False
measuring_status = "0"

laser_pointers_status = "0"
laser_pointers_for_measurement_state = False

ultra_distance_list = []
laser_distance_list = []
time_string = ""
time_string_list = []

ultra_distance = 0  # Ultrasonic sensor distance
laser_distance = 0  # Laser sensor distance
diff_ultra_distance = 0  # Range of ultrasonic distances
diff_laser_distance = 0  # Range of laser distances
prev_ultra_distance = 0  # Previous ultrasonic distance
prev_laser_distance = 0  # Previous laser distance
gyro = 0  # Gyroscope reading
time_from_arduino = ""  # Time received from Arduino as a string
sd_present = "-"  # SD card status as a string
range_interval_ultra_distance = 0  # Range interval for ultrasonic distances
ultra_distance_stddev = 0  # Standard deviation of ultrasonic distances
range_interval_laser_distance = 0  # Range interval for laser distances
laser_distance_stddev = 0
numReadingsPerSecond = "0"

confidence_level = 0.95

array_for_arduino_to_decode = []

battery_lvl = 0


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
    root.after(2000, update_gui_status)

def refresh_ports():
    """Refresh the list of available serial ports and update the combobox."""
    available_ports = get_available_ports()  # Get updated list of ports
    port_combobox['values'] = available_ports  # Update the combobox with new ports


def update_gui_status():
    """Update the GUI to reflect the connection status."""
    global connection, measuring_status, numReadingsPerSecond
    if arduinoData:
        """while arduinoData.in_waiting > 0:
            data = arduinoData.readline().decode('utf-8').strip().split(":")
            if len(data) == 5:
                numReadingsPerSecond = str(data[1])
                readings_analysis_val.config(text=numReadingsPerSecond)
                break """
        status_label.config(text="Connection: Established", foreground="green")
        select_button.config(state=tk.DISABLED)
        refresh_button.config(state=tk.DISABLED)
        start_button.config(state=tk.NORMAL)
        close_button.config(state=tk.NORMAL)
        laser_pointer_button.config(state=tk.NORMAL)

        connection = True
    else:
        status_label.config(text="Connection: Failed", foreground="red")


def start_measuring():
    """Start measuring for both sensors."""
    global measuring, arduinoData, measuring_status
    if not measuring:
       # arduinoData.reset_input_buffer()  # https://stackoverflow.com/questions/60766714/pyserial-flush-vs-reset-input-buffer-reset-output-buffer
        measuring = True
        measuring_status = "1"
        threading.Thread(target=read_sensor_data, daemon=True).start()
        write_to_arduino()
    update_start_measurement_ui()

def stop_measuring():
    """Stop measuring for both sensors."""
    global measuring, arduinoData, measuring_status
    arduinoData.reset_input_buffer()  # https://stackoverflow.com/questions/60766714/pyserial-flush-vs-reset-input-buffer-reset-output-buffer
    measuring = False
    measuring_status = "0"
    write_to_arduino()
    sd_value.config(text="-")
    battery_value.config(text="-")
    update_stop_measurement_ui()

def read_sensor_data():
    global measuring, arduinoData, connection, time_string
    global ultra_distance, diff_ultra_distance, prev_ultra_distance, range_interval_ultra_distance, ultra_distance_stddev
    global laser_distance, diff_laser_distance, prev_laser_distance, range_interval_laser_distance, laser_distance_stddev
    global sd_present, gyro, time_from_arduino, numReadingsPerSecond, confidence_level,battery_lvl
    if connection and arduinoData:
        try:
            while measuring:
                if arduinoData.in_waiting > 0:
                    data = arduinoData.readline().decode('utf-8').strip().split(
                        ",")  # by Paul McWhorter 25 Jan 2022 https://www.youtube.com/watch?v=VN3HJm3spRE&ab_channel=PaulMcWhorter
                    """if isinstance(data[0], str):
                        numReadingsPerSecond = (str(data[0]))
                        data = data[1:]
                        readings_analysis_val.config(text=numReadingsPerSecond)
                        print(len(data))
                        print(data)"""

                    numReadingsPerSecond = "25"
                    readings_analysis_val.config(text=numReadingsPerSecond)

                    if len(data[1:]) == int(numReadingsPerSecond)*2 +4:
                        print(data)
                        ultra_list = [float(value) for value in data[1:int(numReadingsPerSecond)]]
                        laser_list = [float(value) for value in
                                      data[int(numReadingsPerSecond):int(numReadingsPerSecond) * 2]]

                        alpha = 1 - confidence_level  # Calculate alpha (significance level)

                        df_ultra = len(ultra_list) - 1  # Degrees of freedom
                        t_critical_ultra = st.t.ppf(1 - alpha / 2,
                                              df_ultra)  # Two-tailed t-critical value https://www.geeksforgeeks.org/how-to-find-the-t-critical-value-in-python/

                        df_laser = len(laser_list) - 1  # Degrees of freedom
                        t_critical_laser = st.t.ppf(1 - alpha/ 2,
                                              df_laser)  # Two-tailed t-critical value https://www.geeksforgeeks.org/how-to-find-the-t-critical-value-in-python/

                        ultra_distance_before = round(np.mean(ultra_list), 4)  # mean before
                        range_interval_ultra_distance = round(max(ultra_list) - min(ultra_list), 4)
                        ultra_distance_stddev = round(np.std(ultra_list,ddof=1), 4)

                        ultra_ci_ebm = t_critical_ultra * (ultra_distance_stddev / np.sqrt(len(ultra_list)))
                        ultra_lower_ci = ultra_distance - ultra_ci_ebm
                        ultra_upper_ci = ultra_distance + ultra_ci_ebm
                        ultra_ci_ebm = round(ultra_ci_ebm, 4)
                        ultra_lower_ci = round(ultra_lower_ci, 4)
                        ultra_upper_ci = round(ultra_upper_ci, 4)

                        ultrasonic_range_val.config(text=range_interval_ultra_distance)
                        ultra_ci_val.config(text=f"({ultra_lower_ci}, {ultra_upper_ci})")
                        ultra_ci_margin_error_val.config(text=ultra_ci_ebm)
                        ultrasonic_stddev_val.config(text=f"{ultra_distance_stddev}")
                        laser_distance_before = round(np.mean(laser_list), 3)
                        range_interval_laser_distance = round(max(laser_list) - min(laser_list), 2)
                        laser_distance_stddev = round(np.std(laser_list,ddof=1), 4)

                        laser_ci_ebm = t_critical_laser * (laser_distance_stddev / np.sqrt(len(laser_list)))
                        laser_lower_ci = laser_distance - laser_ci_ebm
                        laser_upper_ci = laser_distance + laser_ci_ebm
                        laser_ci_ebm = round(laser_ci_ebm, 4)
                        laser_lower_ci = round(laser_lower_ci, 4)
                        laser_upper_ci = round(laser_upper_ci, 4)

                        laser_range_val.config(text=range_interval_laser_distance)
                        laser_stddev_val.config(text=f"{laser_distance_stddev}")
                        laser_ci_val.config(text=f"({laser_lower_ci}, {laser_upper_ci})")
                        laser_ci_margin_error_val.config(text=laser_ci_ebm)

                        ultra_list_filtered = remove_outliers(ultra_list)  # filter outliers with IQR
                        laser_list_filtered = remove_outliers(laser_list)

                        df_ultra = len(ultra_list_filtered) - 1  # Degrees of freedom
                        t_critical_ultra = st.t.ppf(1 - alpha / 2,
                                                    df_ultra)  # Two-tailed t-critical value https://www.geeksforgeeks.org/how-to-find-the-t-critical-value-in-python/

                        df_laser = len(laser_list_filtered) - 1  # Degrees of freedom
                        t_critical_laser = st.t.ppf(1 - alpha / 2,
                                                    df_laser)  # Two-tailed t-critical value https://www.geeksforgeeks.org/how-to-find-the-t-critical-value-in-python/

                        ultra_distance = round(np.mean(ultra_list_filtered), 4)  # mean after
                        range_interval_ultra_distance = round(max(ultra_list_filtered) - min(ultra_list_filtered), 3)
                        ultra_distance_stddev = round(np.std(ultra_list_filtered,ddof=1), 4)

                        ultra_ci_ebm = t_critical_ultra * (ultra_distance_stddev / np.sqrt(len(ultra_list_filtered)))
                        ultra_lower_ci = ultra_distance - ultra_ci_ebm
                        ultra_upper_ci = ultra_distance + ultra_ci_ebm
                        ultra_ci_ebm = round(ultra_ci_ebm, 4)
                        ultra_lower_ci = round(ultra_lower_ci, 4)
                        ultra_upper_ci = round(ultra_upper_ci, 4)

                        ultrasonic_range_val_after.config(text=range_interval_ultra_distance)
                        ultra_ci_val_after.config(text=f"({ultra_lower_ci}, {ultra_upper_ci})")
                        ultra_ci_margin_error_val_after.config(text=ultra_ci_ebm)
                        ultrasonic_stddev_val_after.config(text=f"{ultra_distance_stddev}")

                        ultrasonic_distance_label_value.config(text=f"{ultra_distance}")
                        ultrasonic_previous_value_label_value.config(text=prev_ultra_distance)
                        diff_ultra_distance = round(ultra_distance - prev_ultra_distance, 4)
                        ultrasonic_diff_label_value.config(text=diff_ultra_distance)

                        laser_distance = round(np.mean(laser_list_filtered), 3)
                        range_interval_laser_distance = round(max(laser_list_filtered) - min(laser_list_filtered), 2)
                        laser_distance_stddev = round(np.std(laser_list_filtered,ddof=1), 4)

                        laser_ci_ebm = t_critical_laser * (laser_distance_stddev / np.sqrt(len(laser_list_filtered)))
                        laser_lower_ci = laser_distance - laser_ci_ebm
                        laser_upper_ci = laser_distance + laser_ci_ebm
                        laser_ci_ebm = round(laser_ci_ebm, 4)
                        laser_lower_ci = round(laser_lower_ci, 4)
                        laser_upper_ci = round(laser_upper_ci, 4)

                        laser_range_val_after.config(text=range_interval_laser_distance)
                        laser_stddev_val_after.config(text=f"{laser_distance_stddev}")
                        laser_ci_val_after.config(text=f"({laser_lower_ci}, {laser_upper_ci})")
                        laser_ci_margin_error_val_after.config(text=laser_ci_ebm)

                        laser_distance_label_value.config(text=f"{laser_distance}")
                        laser_previous_value_label_value.config(text=prev_laser_distance)
                        diff_laser_distance = round(laser_distance - prev_laser_distance, 3)
                        laser_diff_label_value.config(text=diff_laser_distance)

                        gyro = float(str(data[(int(numReadingsPerSecond) * 2)+1]))
                        gyro_label_value.config(text=gyro)
                        time_from_arduino = str(data[(int(numReadingsPerSecond) * 2)+2])
                        #time_from_arduino = strftime("%H:%M:%S")  # by Bro Code 16 Sept 2020  https://www.youtube.com/watch?v=l7IMBy4_nhA&t=183s&ab_channel=BroCode

                        time_string_list.append(time_from_arduino)
                        ultra_distance_list.append(round(ultra_distance, 2))
                        laser_distance_list.append(round(laser_distance, 2))
                        plot_graph_update()

                        sd_present = str((data[(int(numReadingsPerSecond) * 2) + 3]))
                        sd_value.config(text=sd_present)

                        battery_lvl = int(str((data[(int(numReadingsPerSecond) * 2) + 4])))
                        if battery_lvl > 20:
                            battery_value.config(text=f"{battery_lvl}", foreground="black")
                        else:
                            battery_value.config(text=f"{battery_lvl}", foreground="red")

                        ultrasonic_update_sensor_data(ultra_distance)
                        laser_update_sensor_data(laser_distance)

                        prev_ultra_distance = ultra_distance
                        prev_laser_distance = laser_distance

                        ultra_distance = int(ultra_distance)
                        laser_distance = int(laser_distance)

                        write_to_arduino()

                        ultra_list.clear()
                        laser_list.clear()

        except serial.SerialException as e:
            print(f"Error with serial connection: {e}")
            connection = False
        except Exception as e:
            print(f"Error reading sensor data: {e}")

def remove_outliers(data):
    Q1 = np.percentile(data, 25)
    Q3 = np.percentile(data, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR

    # Removing outliers
    filtered_data = [x for x in data if (x >= lower_bound) and (
            x <= upper_bound)]  # https://medium.com/@nirajan.acharya777/understanding-outlier-removal-using-interquartile-range-iqr-b55b9726363e
    return filtered_data

def ultrasonic_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 700:
        status = "Hogging!"
    elif distance < 700:
        status = "Sagging!"
    elif distance == 700:
        status = "OK!"
    ultrasonic_hogging_label_value.config(text=status)

def laser_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 700:
        status = "Hogging!"
    elif distance < 700:
        status = "Sagging!"
    elif distance == 700:
        status = "OK!"
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
    global arduinoData, connection, laser_distance_list, ultra_distance_list, time_string_list, laser_pointers_for_measurement_state
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
            laser_pointer_button.config(state=tk.DISABLED)
            ultrasonic_distance_label_value.config(text=0)
            ultrasonic_diff_label_value.config(text=0)
            ultrasonic_hogging_label_value.config(text="OK")
            ultrasonic_previous_value_label_value.config(text=0)
            laser_distance_label_value.config(text=0)
            laser_diff_label_value.config(text=0)
            laser_hogging_label_value.config(text="OK")
            laser_previous_value_label_value.config(text=0)
            gyro_label_value.config(text=0)
            readings_analysis_val.config(text=0)
            laser_range_val.config(text=0)
            ultra_ci_val.config(text="(0, 0)")
            ultra_ci_margin_error_val.config(text=0)
            laser_ci_val.config(text="(0, 0)")
            laser_ci_margin_error_val.config(text=0)
            laser_stddev_val.config(text=0)
            ultrasonic_range_val.config(text=0)
            ultrasonic_stddev_val.config(text=0)
            laser_range_val_after.config(text=0)
            ultra_ci_val_after.config(text="(0, 0)")
            ultra_ci_margin_error_val_after.config(text=0)
            laser_ci_val_after.config(text="(0, 0)")
            laser_ci_margin_error_val_after.config(text=0)
            laser_stddev_val_after.config(text=0)
            ultrasonic_range_val_after.config(text=0)
            ultrasonic_stddev_val_after.config(text=0)
            ultra_distance_list.clear()
            laser_distance_list.clear()
            time_string_list.clear()
            plot_graph_update()
            laser_pointers_for_measurement_state = False
            laser_pointer_button.config(text="Laser pointers (OFF)", foreground="red")


def plot_graph_update():
    global ultra_distance_list, laser_distance_list, time_string_list, time_string

    ax.clear()  # Clear the axes
    ax.plot(time_string_list, ultra_distance_list, marker='x', markersize=5, label='Ultrasonic Distance')
    ax.plot(time_string_list, laser_distance_list, marker='o', markersize=5, label='Laser Distance')

    if len(ultra_distance_list) > 10:  # if array size >10, pop the first index of all 3 arrays https://www.programiz.com/python-programming/methods/list/pop
        ultra_distance_list = ultra_distance_list[-10:]
        laser_distance_list = laser_distance_list[-10:]
        time_string_list = time_string_list[-10:]

    ax.set_title("Time against Distance")
    ax.set_ylabel("Distance (mm)")
    ax.legend()
    canvas.draw()


def laser_pointers_for_measurement():
    global laser_pointers_status, laser_pointers_for_measurement_state
    laser_pointers_for_measurement_state = not laser_pointers_for_measurement_state
    if laser_pointers_for_measurement_state:
        laser_pointers_status = "1"
        laser_pointers_for_measurement_state = True
        laser_pointer_button.config(text="Laser pointers (ON)", foreground="green", )
    if not laser_pointers_for_measurement_state:
        laser_pointers_status = "0"
        laser_pointers_for_measurement_state = False
        laser_pointer_button.config(text="Laser pointers (OFF)", foreground="red")
    write_to_arduino()

def write_to_arduino():
    global measuring_status, laser_pointers_status, ultra_distance,laser_distance
    data_to_send = measuring_status + laser_pointers_status + ":" +  str(
        ultra_distance) + ";" + str(
        laser_distance) + "\n"
    arduinoData.write(data_to_send.encode("utf-8"))
    arduinoData.flush()

    print("Sent data:", data_to_send)

# GUI Setup
root = tk.Tk()  # by Bro Code 22 Sept 2020 https://www.youtube.com/watch?v=lyoyTlltFVU&ab_channel=BroCode
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1200x1000")

"""data_label = tk.LabelFrame(root, text=date_string,font=("Arial", 10))
data_label.pack()
"""
# ComboBox for port selection
port_combobox = ttk.Combobox(root, values=get_available_ports())  # Chatgpt
port_combobox.pack(pady=1)  # Place Combobox in first column

port_frame = tk.Frame(root)
port_frame.pack(pady=5, padx=5)  # Add some padding around the frame

select_button = ttk.Button(port_frame, text="Select Port", command=lambda: on_select())
select_button.grid(row=0, column=0, padx=5)  # Place Button in second column

refresh_button = ttk.Button(port_frame, text="Refresh Ports", command=refresh_ports)
refresh_button.grid(row=0, column=1, padx=5)  # Place Button in second column

close_button = ttk.Button(root, text="Close Connection", command=close_serial, state=tk.DISABLED)
close_button.pack(pady=1)

# Connection status label
status_label = ttk.Label(root, text="Connection: Not Established", foreground="red")
status_label.pack(pady=1)

laser_pointer_button = tk.Button(root, text="Laser pointers (OFF)", command=laser_pointers_for_measurement,
                                 foreground="red", bg="white", relief="flat", state=tk.DISABLED)
laser_pointer_button.pack(pady=1)

measuring_frame = tk.Frame(root)
measuring_frame.pack(pady=5, padx=5)  # Add some padding around the frame
# Start/Stop buttons
start_button = ttk.Button(measuring_frame, text="Start Measuring", command=start_measuring, state=tk.DISABLED)
start_button.grid(row=0, column=0, padx=5)

stop_button = ttk.Button(measuring_frame, text="Stop Measuring", command=stop_measuring, state=tk.DISABLED)
stop_button.grid(row=0, column=1, padx=5)

sd = tk.Frame(root)
sd.pack()

# LabelFrame for SD card details
sd_frame = tk.LabelFrame(sd, text="SD Card", padx=5, pady=5)
sd_frame.grid(row=0, column=1)  # Position SD card frame to the right (column 1)
# Label for "Saving Data?" text
sd_label = tk.Label(sd_frame, text="Saving Data?:", font=("Arial", 9))
sd_label.pack()
sd_value = tk.Label(sd_frame, text=sd_present, font=("Arial", 9))
sd_value.pack()

# LabelFrame for Battery details
battery_frame = tk.LabelFrame(sd, text="Battery", padx=5, pady=5)
battery_frame.grid(row=0, column=0)  # Position battery frame to the left (column 0)
# Label for "Battery Voltage" text
battery_label = tk.Label(battery_frame, text="Battery(%):", font=("Arial", 9))
battery_label.pack()
battery_value = tk.Label(battery_frame, text="-", font=("Arial", 9))
battery_value.pack()

analysis = tk.Frame(root)
analysis.pack()
analysis_frame = tk.LabelFrame(analysis, text="Analysis", padx=5, pady=5)
analysis_frame.pack()

# Centralized "Readings per Second" in the first row
readings_analysis = tk.Label(analysis_frame, text="Readings per Second:", padx=5, pady=5, font=("Arial", 10))
readings_analysis.grid(row=0, column=0, pady=1, )  # Span across two columns and center it
readings_analysis_val = tk.Label(analysis_frame, text=numReadingsPerSecond, padx=5, pady=5, font=("Arial", 10))
readings_analysis_val.grid(row=0, column=1, pady=1)  # Span across two columns and center it

# Ultrasonic Sensor Frame
ultrasonic_analysis_frame = tk.LabelFrame(analysis_frame, text="Ultrasonic Sensor", padx=5, pady=5)
ultrasonic_analysis_frame.grid(row=1, column=0, padx=30, pady=10)  # Place in row 1, column 0

ultrasonic_range_label = tk.Label(ultrasonic_analysis_frame, text="Range:", font=("Arial", 10))
ultrasonic_range_label.grid(row=0, column=0, padx=5, pady=5)
ultrasonic_stddev_label = tk.Label(ultrasonic_analysis_frame, text="StdDev:", font=("Arial", 10))
ultrasonic_stddev_label.grid(row=0, column=1, padx=5, pady=5)
ultra_ci_label = tk.Label(ultrasonic_analysis_frame, text=f"{int(confidence_level * 100)}% CI:", font=("Arial", 10))
ultra_ci_label.grid(row=0, column=2, padx=5, pady=5)
ultra_ci_margin_error_label = tk.Label(ultrasonic_analysis_frame, text="Margin Error:", font=("Arial", 10))
ultra_ci_margin_error_label.grid(row=0, column=3, padx=5, pady=5)

ultrasonic_range_val = tk.Label(ultrasonic_analysis_frame, text=range_interval_ultra_distance, font=("Arial", 10))
ultrasonic_range_val.grid(row=1, column=0, padx=5, pady=5)
ultrasonic_stddev_val = tk.Label(ultrasonic_analysis_frame, text=ultra_distance_stddev, font=("Arial", 10))
ultrasonic_stddev_val.grid(row=1, column=1, padx=5, pady=5)
ultra_ci_val = tk.Label(ultrasonic_analysis_frame, text="(0, 0)", font=("Arial", 10))
ultra_ci_val.grid(row=1, column=2, padx=5, pady=5)
ultra_ci_margin_error_val = tk.Label(ultrasonic_analysis_frame, text="0", font=("Arial", 10))
ultra_ci_margin_error_val.grid(row=1, column=3, padx=5, pady=5)
ultra_analysis_before = tk.Label(ultrasonic_analysis_frame, text="(Before IQR)", font=("Arial", 10))
ultra_analysis_before.grid(row=1, column=4, padx=5, pady=5)

ultrasonic_range_val_after = tk.Label(ultrasonic_analysis_frame, text=range_interval_ultra_distance, font=("Arial", 10))
ultrasonic_range_val_after.grid(row=2, column=0, padx=5, pady=5)
ultrasonic_stddev_val_after = tk.Label(ultrasonic_analysis_frame, text=ultra_distance_stddev, font=("Arial", 10))
ultrasonic_stddev_val_after.grid(row=2, column=1, padx=5, pady=5)
ultra_ci_val_after = tk.Label(ultrasonic_analysis_frame, text="(0, 0)", font=("Arial", 10))
ultra_ci_val_after.grid(row=2, column=2, padx=5, pady=5)
ultra_ci_margin_error_val_after = tk.Label(ultrasonic_analysis_frame, text="0", font=("Arial", 10))
ultra_ci_margin_error_val_after.grid(row=2, column=3, padx=5, pady=5)
ultra_analysis_after = tk.Label(ultrasonic_analysis_frame, text="(After IQR)", font=("Arial", 10))
ultra_analysis_after.grid(row=2, column=4, padx=5, pady=5)

# Laser Sensor Frame
laser_analysis_frame = tk.LabelFrame(analysis_frame, text="Laser Sensor", padx=5, pady=5)
laser_analysis_frame.grid(row=1, column=1, padx=30, pady=10)  # Place in row 1, column 1

laser_range_label = tk.Label(laser_analysis_frame, text="Range:", font=("Arial", 10))
laser_range_label.grid(row=0, column=0, padx=5, pady=5)
laser_stddev_label = tk.Label(laser_analysis_frame, text="StdDev:", font=("Arial", 10))
laser_stddev_label.grid(row=0, column=1, padx=5, pady=5)
laser_ci_label = tk.Label(laser_analysis_frame, text=f"{int(confidence_level * 100)}% CI:", font=("Arial", 10))
laser_ci_label.grid(row=0, column=2, padx=5, pady=5)
laser_ci_margin_error_label = tk.Label(laser_analysis_frame, text="Margin Error:", font=("Arial", 10))
laser_ci_margin_error_label.grid(row=0, column=3, padx=5, pady=5)

laser_range_val = tk.Label(laser_analysis_frame, text=range_interval_laser_distance, font=("Arial", 10))
laser_range_val.grid(row=1, column=0, padx=5, pady=5)
laser_stddev_val = tk.Label(laser_analysis_frame, text=laser_distance_stddev, font=("Arial", 10))
laser_stddev_val.grid(row=1, column=1, padx=5, pady=5)
laser_ci_val = tk.Label(laser_analysis_frame, text="(0, 0)", font=("Arial", 10))
laser_ci_val.grid(row=1, column=2, padx=5, pady=5)
laser_ci_margin_error_val = tk.Label(laser_analysis_frame, text="0", font=("Arial", 10))
laser_ci_margin_error_val.grid(row=1, column=3, padx=5, pady=5)
laser_analysis_before = tk.Label(laser_analysis_frame, text="(Before IQR)", font=("Arial", 10))
laser_analysis_before.grid(row=1, column=4, padx=5, pady=5)

laser_range_val_after = tk.Label(laser_analysis_frame, text=range_interval_laser_distance, font=("Arial", 10))
laser_range_val_after.grid(row=2, column=0, padx=5, pady=5)
laser_stddev_val_after = tk.Label(laser_analysis_frame, text=laser_distance_stddev, font=("Arial", 10))
laser_stddev_val_after.grid(row=2, column=1, padx=5, pady=5)
laser_ci_val_after = tk.Label(laser_analysis_frame, text="(0, 0)", font=("Arial", 10))
laser_ci_val_after.grid(row=2, column=2, padx=5, pady=5)
laser_ci_margin_error_val_after = tk.Label(laser_analysis_frame, text="0", font=("Arial", 10))
laser_ci_margin_error_val_after.grid(row=2, column=3, padx=5, pady=5)
laser_analysis_after = tk.Label(laser_analysis_frame, text="(After IQR)", font=("Arial", 10))
laser_analysis_after.grid(row=2, column=4, padx=5, pady=5)

sensor = tk.Frame(root)
sensor.pack(pady=10)

main_container = tk.LabelFrame(sensor, text="Sensors", padx=10, pady=10)
main_container.pack(side="left", padx=10)

ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=30)  # Use grid layout for better control

ultrasonic_distance_label = tk.Label(ultrasonic_frame, text="Mean Distance (mm):", font=("Arial", 12))
ultrasonic_distance_label.grid(row=0, column=0, pady=5, sticky="w")
ultrasonic_distance_label_value = tk.Label(ultrasonic_frame, text=ultra_distance, font=("Arial", 12))
ultrasonic_distance_label_value.grid(row=0, column=1, pady=5)

ultrasonic_previous_value_label = tk.Label(ultrasonic_frame, text="Previous:", font=("Arial", 12))
ultrasonic_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
ultrasonic_previous_value_label_value = tk.Label(ultrasonic_frame, text=prev_ultra_distance, font=("Arial", 12))
ultrasonic_previous_value_label_value.grid(row=1, column=1, pady=5)

ultrasonic_diff_label = tk.Label(ultrasonic_frame, text="Diff (mm/s):", font=("Arial", 12))
ultrasonic_diff_label.grid(row=2, column=0, pady=5, sticky="w")
ultrasonic_diff_label_value = tk.Label(ultrasonic_frame, text=diff_ultra_distance, font=("Arial", 12))
ultrasonic_diff_label_value.grid(row=2, column=1, pady=5)

ultrasonic_hogging_label = tk.Label(ultrasonic_frame, text="Status:", font=("Arial", 12))
ultrasonic_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
ultrasonic_hogging_label_value = tk.Label(ultrasonic_frame, text="OK", font=("Arial", 12))
ultrasonic_hogging_label_value.grid(row=3, column=1, pady=5)

# Laser sensor frame
laser_frame = tk.LabelFrame(main_container, text="Laser Sensor", padx=10, pady=10)
laser_frame.grid(row=0, column=1, padx=30)  # Place laser frame next to ultrasonic sensor frame

laser_distance_label = tk.Label(laser_frame, text="Mean Distance (mm):", font=("Arial", 12))
laser_distance_label.grid(row=0, column=0, pady=5, sticky="w")
laser_distance_label_value = tk.Label(laser_frame, text=laser_distance, font=("Arial", 12))
laser_distance_label_value.grid(row=0, column=1, pady=5)

laser_previous_value_label = tk.Label(laser_frame, text="Previous:", font=("Arial", 12))
laser_previous_value_label.grid(row=1, column=0, pady=5, sticky="w")
laser_previous_value_label_value = tk.Label(laser_frame, text=prev_laser_distance, font=("Arial", 12))
laser_previous_value_label_value.grid(row=1, column=1, pady=5)

laser_diff_label = tk.Label(laser_frame, text="Diff (mm/s):", font=("Arial", 12))
laser_diff_label.grid(row=2, column=0, pady=5, sticky="w")
laser_diff_label_value = tk.Label(laser_frame, text=diff_laser_distance, font=("Arial", 12))
laser_diff_label_value.grid(row=2, column=1, pady=5)

laser_hogging_label = tk.Label(laser_frame, text="Status:", font=("Arial", 12))
laser_hogging_label.grid(row=3, column=0, pady=5, sticky="w")
laser_hogging_label_value = tk.Label(laser_frame, text="OK", font=("Arial", 12))
laser_hogging_label_value.grid(row=3, column=1, pady=5)

gyro_frame = tk.LabelFrame(main_container, text="Gyro Sensor", padx=10, pady=10)
gyro_frame.grid(row=0, column=2, padx=10)

gyro_label = tk.Label(gyro_frame, text="Gyro X (deg/s):", font=("Arial", 12))
gyro_label.grid(row=0, column=0, pady=5, sticky="w")
gyro_label_value = tk.Label(gyro_frame, text=gyro, font=("Arial", 12))
gyro_label_value.grid(row=0, column=1, pady=5)

fig, ax = plt.subplots()  # CodersLegacy  21 Feb 2023 https://youtu.be/lVTC8CvScQo?si=TdkXePkDE8nejzmu
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True, )

ax.set_title("Time against Distance")
ax.set_ylabel("Distance (mm)")
canvas.draw()

# clock_update()

# plot_graph_update()
# Start the Tkinter event loop

root.mainloop()
