import tkinter as tk

import serial
import threading
import serial.tools.list_ports
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Configuration Constants
arduino_BAUD_RATE = 115200
connection = False
# Sensor Variables
measuring = False
measuring_status = "STOP"

laser_pointers_status = "OFF"
laser_pointers_for_measurement_state = False

ultra_distance_list=[]
laser_distance_list =[]
time_string = ""
time_string_list=[]

ultra_distance = 0                  # Ultrasonic sensor distance
laser_distance = 0                  # Laser sensor distance
diff_ultra_distance = 0            # Range of ultrasonic distances
diff_laser_distance = 0            # Range of laser distances
prev_ultra_distance = 0             # Previous ultrasonic distance
prev_laser_distance = 0             # Previous laser distance
gyro = 0                            # Gyroscope reading
time_from_arduino = ""                # Time received from Arduino as a string
sd_present = "No"                       # SD card status as a string
range_interval_ultra_distance = 0   # Range interval for ultrasonic distances
ultra_distance_var = 0              # Variance of ultrasonic distances
ultra_distance_stddev = 0           # Standard deviation of ultrasonic distances
range_interval_laser_distance = 0   # Range interval for laser distances
laser_distance_var = 0              # Variance of laser distances
laser_distance_stddev = 0
numReadingsPerSecond = "0"

array_for_arduino_to_decode=[]

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
    root.after(1000, update_gui_status)

def refresh_ports():
    """Refresh the list of available serial ports and update the combobox."""
    available_ports = get_available_ports()  # Get updated list of ports
    port_combobox['values'] = available_ports  # Update the combobox with new ports

def update_gui_status():
    """Update the GUI to reflect the connection status."""
    global connection, measuring_status
    if arduinoData:
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
        measuring = True
        measuring_status = "START"
        threading.Thread(target=read_sensor_data, daemon=True).start()
        write_to_arduino()
    root.after(0, update_start_measurement_ui)

def stop_measuring():
    """Stop measuring for both sensors."""
    global measuring, arduinoData,measuring_status
    arduinoData.reset_input_buffer()  # https://stackoverflow.com/questions/60766714/pyserial-flush-vs-reset-input-buffer-reset-output-buffer
    measuring = False
    measuring_status = "STOP"
    write_to_arduino()
    root.after(0, update_stop_measurement_ui)

def read_sensor_data():
    global measuring, arduinoData, connection,time_string
    global ultra_distance,diff_ultra_distance,prev_ultra_distance,range_interval_ultra_distance,ultra_distance_var,ultra_distance_stddev
    global laser_distance,diff_laser_distance,prev_laser_distance,range_interval_laser_distance,laser_distance_var,laser_distance_stddev
    global sd_present,gyro,time_from_arduino,numReadingsPerSecond
    if connection and arduinoData:
        try:
            while measuring:
                if arduinoData.in_waiting > 0:
                    data = arduinoData.readline().decode('utf-8').strip().split(",")    #by Paul McWhorter 25 Jan 2022 https://www.youtube.com/watch?v=VN3HJm3spRE&ab_channel=PaulMcWhorter
                    if len(data) ==16:
                        ultra_distance = float(data[0])
                        laser_distance = float(data[1])
                        diff_ultra_distance = float(data[2])
                        diff_laser_distance = float(data[3])
                        prev_ultra_distance = float(data[4])
                        prev_laser_distance = float(data[5])
                        gyro = float(data[6])
                        time_from_arduino = str(data[7])
                        sd_present = str(data[8])
                        range_interval_ultra_distance = float(data[9])
                        ultra_distance_var = float(data[10])
                        ultra_distance_stddev = float(data[11])
                        range_interval_laser_distance = float(data[12])
                        laser_distance_var = float(data[13])
                        laser_distance_stddev = float(data[14])
                        numReadingsPerSecond = str(data[15])

                        # Update Ultrasonic Sensor Data
                        root.after(0, ultrasonic_update_distance_label, ultra_distance)
                        root.after(0, laser_update_distance_label, laser_distance)
                        root.after(0, ultrasonic_calculate_and_update_range, prev_ultra_distance, diff_ultra_distance)
                        root.after(0, laser_calculate_and_update_range, prev_laser_distance, diff_laser_distance)

                        ultrasonic_update_sensor_data(ultra_distance)
                        laser_update_sensor_data(laser_distance)

                        laser_distance_rounded = round(laser_distance, 2)
                        ultra_distance_rounded = round(ultra_distance, 2)

                        ultra_distance_list.append(ultra_distance_rounded)
                        laser_distance_list.append(laser_distance_rounded)
                        time_string_list.append(time_from_arduino)

                        readings_analysis_val.config(text=numReadingsPerSecond)
                        laser_range_val.config(text=range_interval_laser_distance)
                        laser_variance_val.config(text=laser_distance_var)
                        laser_stddev_val.config(text=laser_distance_stddev)
                        ultrasonic_range_val.config(text=range_interval_ultra_distance)
                        ultrasonic_variance_val.config(text=ultra_distance_var)
                        ultrasonic_stddev_val.config(text=ultra_distance_stddev)

                        # Update Gyro Sensor Data
                        update_gyro_label(gyro)  # Update the gyro value separately

                        plot_graph_update()

                        update_sd_label(sd_present)

                        write_to_arduino()



        except serial.SerialException as e:
            print(f"Error with serial connection: {e}")
            connection = False
        except Exception as e:
            print(f"Error reading sensor data: {e}")


def update_sd_label(sd_present_val):
    sd_value.config(text=sd_present_val)

def update_gyro_label(gyro_value):
    gyro_label_value.config(text=gyro_value)

def ultrasonic_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 700:
        status = "Hogging!"
    elif distance < 700:
        status = "Sagging!"
    elif distance == 700:
        status = "OK!"
    root.after(0, ultrasonic_update_hogging_status, status)

def laser_update_sensor_data(distance):
    status = "OK!"
    # Check for hogging/sagging status; used as 10cm original displacement between two objects for reference
    if distance > 700:
        status = "Hogging!"
    elif distance < 700:
        status = "Sagging!"
    elif distance == 700:
        status = "OK!"
    root.after(0, laser_update_hogging_status, status)

def ultrasonic_update_distance_label(distance):
    """Update distance label on GUI based on sensor type."""
    ultrasonic_distance_label_value.config(text=distance)

def laser_update_distance_label(distance):
    """Update distance label on GUI based on sensor type."""
    laser_distance_label_value.config(text=distance)

def ultrasonic_update_hogging_status(status):
    """Update the hogging status on the GUI based on sensor type."""
    ultrasonic_hogging_label_value.config(text=status)

def laser_update_hogging_status(status):
    """Update the hogging status on the GUI based on sensor type."""
    laser_hogging_label_value.config(text=status)

def ultrasonic_calculate_and_update_range(previous_value, range_value):
    """Calculate and update the range difference."""
    ultrasonic_diff_label_value.config(text=range_value)
    ultrasonic_previous_value_label_value.config(text=previous_value)

def laser_calculate_and_update_range(previous_value, range_value):
    """Calculate and update the range difference."""
    laser_diff_label_value.config(text=range_value)
    laser_previous_value_label_value.config(text=previous_value)

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
    global arduinoData, connection,laser_distance_list,ultra_distance_list,time_string_list
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
            ultra_distance_list.clear()
            laser_distance_list.clear()
            time_string_list.clear()
            plot_graph_update()


#def clock_update():
 #   global time_string_list, time_string
 #   time_string = strftime("%H:%M:%S") #by Bro Code 16 Sept 2020  https://www.youtube.com/watch?v=l7IMBy4_nhA&t=183s&ab_channel=BroCode
 #   time_label.config(text=time_string)

 #   date_string = strftime("%a, %d %b %Y")
 #   date_label.config(text=date_string)
 #   root.after(1000,clock_update)

def plot_graph_update():
    global ultra_distance_list, laser_distance_list, time_string_list, time_string

    ax.clear()  # Clear the axes
    ax.plot(time_string_list, ultra_distance_list, marker='x', markersize=5, label='Ultrasonic Distance')
    ax.plot(time_string_list, laser_distance_list, marker='o', markersize=5, label='Laser Distance')

    if len(ultra_distance_list) > 10:  # if array size >10, pop the first index of all 3 arrays https://www.programiz.com/python-programming/methods/list/pop
        ultra_distance_list = ultra_distance_list[-10:]
        laser_distance_list = laser_distance_list[-10:]
        time_string_list = time_string_list[-10:]

    ax.set_title("Distance over Time")
    ax.set_xlabel("Time (HH:MM:SS)")
    ax.set_ylabel("Distance (mm)")
    ax.legend()
    canvas.draw()
    #root.after(50, plot_graph_update)

def laser_pointers_for_measurement():
    global laser_pointers_status, laser_pointers_for_measurement_state
    laser_pointers_for_measurement_state = not laser_pointers_for_measurement_state
    if laser_pointers_for_measurement_state:
        laser_pointers_status = "ON"
        laser_pointers_for_measurement_state = True
        laser_pointer_button.config(text="Laser pointers (ON)", foreground="green",)
    if not laser_pointers_for_measurement_state:
        laser_pointers_status = "OFF"
        laser_pointers_for_measurement_state = False
        laser_pointer_button.config(text="Laser pointers (OFF)", foreground="red")
    write_to_arduino()


def write_to_arduino():
    global measuring_status, laser_pointers_status
    data_to_send = measuring_status + ":" + laser_pointers_status + "\n"
    arduinoData.write(data_to_send.encode("utf-8"))
    print("Sent data:", data_to_send)


# GUI Setup
root = tk.Tk() #by Bro Code 22 Sept 2020 https://www.youtube.com/watch?v=lyoyTlltFVU&ab_channel=BroCode
root.title("Ultrasonic and Laser Sensor Control")
root.geometry("1200x1000")

# ComboBox for port selection
port_combobox = ttk.Combobox(root, values=get_available_ports()) #Chatgpt
port_combobox.pack(pady =1)  # Place Combobox in first column

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

laser_pointer_button = tk.Button(root, text="Laser pointers (OFF)", command=laser_pointers_for_measurement, foreground="red", bg="white",  relief="flat", state=tk.DISABLED)
laser_pointer_button.pack(pady=1)

measuring_frame = tk.Frame(root)
measuring_frame.pack(pady=5, padx=5)  # Add some padding around the frame
# Start/Stop buttons
start_button = ttk.Button(measuring_frame, text="Start Measuring", command=start_measuring, state=tk.DISABLED)
start_button.grid(row=0, column=0, padx=5)

stop_button = ttk.Button(measuring_frame, text="Stop Measuring", command=stop_measuring,state=tk.DISABLED)
stop_button.grid(row=0, column=1, padx=5)

sd = tk.Frame(root)
sd.pack()
# LabelFrame for SD card details
sd_frame = tk.LabelFrame(sd, text="SD Card", padx=5, pady=5)
sd_frame.pack()
# Label for "Saving Data?" text
sd_label = tk.Label(sd_frame, text="Saving Data?:", font=("Arial", 9))
sd_label.pack()
# Label to display the value of sd_present
sd_value = tk.Label(sd_frame, text=sd_present, font=("Arial", 9))
sd_value.pack()

analysis = tk.Frame(root)
analysis.pack()
analysis_frame = tk.LabelFrame(analysis, text="Analysis", padx=5, pady=5)
analysis_frame.pack()

# Centralized "Readings per Second" in the first row
readings_analysis = tk.Label(analysis_frame, text="Readings per Second:", padx=5, pady=5, font=("Arial", 10))
readings_analysis.grid(row=0, column=0, pady=1, )  # Span across two columns and center it
readings_analysis_val = tk.Label(analysis_frame, text=numReadingsPerSecond, padx=5, pady=5, font=("Arial", 10))
readings_analysis_val.grid(row=0, column=1, pady=1 )  # Span across two columns and center it

# Ultrasonic Sensor Frame
ultrasonic_analysis_frame = tk.LabelFrame(analysis_frame, text="Ultrasonic Sensor", padx=5, pady=5)
ultrasonic_analysis_frame.grid(row=1, column=0, padx=30, pady=10)  # Place in row 1, column 0

ultrasonic_range_label = tk.Label(ultrasonic_analysis_frame, text="Range:", font=("Arial", 10))
ultrasonic_range_label.grid(row=0, column=0, padx=5, pady=5)
ultrasonic_variance_label = tk.Label(ultrasonic_analysis_frame, text="Var:", font=("Arial", 10))
ultrasonic_variance_label.grid(row=0, column=1, padx=5, pady=5)
ultrasonic_stddev_label = tk.Label(ultrasonic_analysis_frame, text="StdDev:", font=("Arial", 10))
ultrasonic_stddev_label.grid(row=0, column=2, padx=5, pady=5)

ultrasonic_range_val = tk.Label(ultrasonic_analysis_frame, text=range_interval_ultra_distance, font=("Arial", 10))
ultrasonic_range_val.grid(row=1, column=0, padx=5, pady=5)
ultrasonic_variance_val = tk.Label(ultrasonic_analysis_frame, text=ultra_distance_var, font=("Arial", 10))
ultrasonic_variance_val.grid(row=1, column=1, padx=5, pady=5)
ultrasonic_stddev_val = tk.Label(ultrasonic_analysis_frame, text=ultra_distance_stddev, font=("Arial", 10))
ultrasonic_stddev_val.grid(row=1, column=2, padx=5, pady=5)

# Laser Sensor Frame
laser_analysis_frame = tk.LabelFrame(analysis_frame, text="Laser Sensor", padx=5, pady=5)
laser_analysis_frame.grid(row=1, column=1, padx=30, pady=10)  # Place in row 1, column 1

laser_range_label = tk.Label(laser_analysis_frame, text="Range:", font=("Arial", 10))
laser_range_label.grid(row=0, column=0, padx=5, pady=5)
laser_variance_label = tk.Label(laser_analysis_frame, text="Var:", font=("Arial", 10))
laser_variance_label.grid(row=0, column=1, padx=5, pady=5)
laser_stddev_label = tk.Label(laser_analysis_frame, text="StdDev:", font=("Arial", 10))
laser_stddev_label.grid(row=0, column=2, padx=5, pady=5)

laser_range_val = tk.Label(laser_analysis_frame, text=range_interval_laser_distance, font=("Arial", 10))
laser_range_val.grid(row=1, column=0, padx=5, pady=5)
laser_variance_val = tk.Label(laser_analysis_frame, text=laser_distance_var, font=("Arial", 10))
laser_variance_val.grid(row=1, column=1, padx=5, pady=5)
laser_stddev_val = tk.Label(laser_analysis_frame, text=laser_distance_stddev, font=("Arial", 10))
laser_stddev_val.grid(row=1, column=2, padx=5, pady=5)

sensor = tk.Frame(root)
sensor.pack(pady=10)

main_container = tk.LabelFrame(sensor, text="Sensors", padx=10, pady=10)
main_container.pack(side="left", padx=10)

ultrasonic_frame = tk.LabelFrame(main_container, text="Ultrasonic Sensor", padx=10, pady=10)
ultrasonic_frame.grid(row=0, column=0, padx=30)  # Use grid layout for better control

ultrasonic_distance_label = tk.Label(ultrasonic_frame, text="Current Distance (mm):", font=("Arial", 12))
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

laser_distance_label = tk.Label(laser_frame, text="Current Distance (mm):", font=("Arial", 12))
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

fig, ax = plt.subplots() #CodersLegacy  21 Feb 2023 https://youtu.be/lVTC8CvScQo?si=TdkXePkDE8nejzmu
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True,)

ax.set_title("Distance over Time")
ax.set_xlabel("Time (HH:MM:SS)")
ax.set_ylabel("Distance (mm)")
canvas.draw()

#clock_update()

#plot_graph_update()
# Start the Tkinter event loop

root.mainloop()
