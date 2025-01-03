import tkinter as tk
import serial
import random
import time
import serial.tools.list_ports
from tkinter import ttk

# RS232 Configuration

RS232_BAUD_RATE = 38400  # Baud rate for RS232 motor control

connection = False

def get_available_ports():
    """Get all available serial ports."""
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def refresh_ports():
    """Refresh the list of available serial ports and update the combobox."""
    available_ports = get_available_ports()  # Get updated list of ports
    port_combobox['values'] = available_ports  # Update the combobox with new ports

motor_serial = None  # COM port for RS232 motor control

def on_select(event):
    """Event handler for port selection."""
    selected_port = port_combobox.get()  # Get the selected port from combobox
    print(f"User selected: {selected_port}")

    # Connect to the selected port
    global motor_serial
    motor_serial = serial.Serial(selected_port, RS232_BAUD_RATE)

    # Start by setting the status to "Establishing"
    status_label.config(text="Connection: Establishing", foreground="orange")
    select_button.config(state=tk.DISABLED)
    refresh_button.config(state=tk.DISABLED)
    root.after(1000, update_gui_status)

def update_gui_status():
    """Update the GUI to reflect the connection status."""
    global connection,motor_serial
    if motor_serial:
        reset_motor_position()
        close_button.config(state=tk.NORMAL)
        connection = True
    else:
        status_label.config(text="Connection: Failed", foreground="red")

def close_serial():
    """Close the serial connection."""
    global motor_serial, connection,total_motor_moves,motor_move_count
    if motor_serial:
        try:
            if motor_serial.is_open:
                motor_serial.close()
                print("Serial connection closed.")
        except Exception as e:
            print(f"Error closing serial connection: {e}")
        finally:
            motor_serial = None
            connection = False
            status_label.config(text="Connection: Not Established", foreground="red")
            total_motor_moves =0
            motor_move_count=0
            # Re-enable the port selection button and disable others
            select_button.config(state=tk.NORMAL)
            refresh_button.config(state=tk.NORMAL)
            close_button.config(state=tk.DISABLED)
            move_motor_button.config(state=tk.DISABLED)
            reset_motor_button.config(state=tk.DISABLED)

            # Reset motor control labels
            position_label_value.config(text="Position: 0")
            initial_position_label_value.config(text="Initial Position: None")
            speed_label_value.config(text="Speed: 0")
            frequency_label_value.config(text="0 moves/min")
            total_moves_label_value.config(text="Total Moves: 0")


motor_move_count = 0  # Counter to keep track of motor movements in time interval
total_motor_moves = 0   # Counter to keep track of total motor movements in the whole running program

current_position = 0
current_speed = 0
previous_position = 0  # Store previous position
initial_position_set = False  # Flag to track if the initial position has been set


last_frequency_update_time = time.time()  # Timestamp of the last frequency update

# Serial Connection
time.sleep(1)

# Function to send ASCII commands to the motor
def send_ascii_command(ser, command):
    """
    Send an ASCII command to the motor via serial communication.
    """
    try:
        ser.write((command + '\r').encode())  # Send command to the motor

        if ser.in_waiting > 0:  # Check if there's a response
            response = ser.read(ser.in_waiting).decode('utf-8').strip()
            print(f"Motor Response: {response}")
    except Exception as e:
        print(f"Error sending motor command: {e}")

# Function to move the motor to a random position at a random speed
def move_motor_randomly():
    """
    Move the motor to a random position at a random speed when the button is pressed.
    """
    global current_position, current_speed, initial_position_set, previous_position, motor_move_count, total_motor_moves, motor_serial


    if motor_serial:
        move_motor_button.config(state=tk.DISABLED)
        reset_motor_button.config(state=tk.DISABLED)
        try:
            # Generate random position and speed for the motor
            position = random.randint(29000, 41000)
            speed = random.randint(50, 250)

            # Update the current position and speed
            current_position = position
            current_speed = speed

            # Update position and speed labels in GUI
            root.after(0, update_position_label, current_position)
            root.after(0, update_speed_label, current_speed)

            print(f"Moving motor to position {position} at speed {speed}...")
            send_ascii_command(motor_serial, f"S.1={speed}")  # Set motor speed
            send_ascii_command(motor_serial, f"P.1={position}")  # Move motor to position
            send_ascii_command(motor_serial, "^.1")  # Start motor movement
            previous_position = current_position

            # Only update initial position when motor moves away from position 0

            motor_move_count += 1
            total_motor_moves += 1  # Increment total moves counter

            update_motor_frequency()
            root.after(0, update_total_moves_label, total_motor_moves)

            if initial_position_set:
                root.after(0, update_initial_position_label, current_position)
                initial_position_set = False

            root.after(300, lambda: move_motor_button.config(state=tk.NORMAL))
            root.after(300, lambda: reset_motor_button.config(state=tk.NORMAL))


        except Exception as e:
            print(f"Motor Error: {e}")
            root.after(300, lambda: move_motor_button.config(state=tk.NORMAL))
            root.after(300, lambda: reset_motor_button.config(state=tk.NORMAL))


# Function to reset motor to position 0
def reset_motor_position():
    """
    Reset the motor position to 0 and update the labels.
    """
    global current_position, current_speed, initial_position_set, motor_serial

    if motor_serial:
        status_label.config(text="Resetting motor...", foreground="orange")
        move_motor_button.config(state=tk.DISABLED)
        reset_motor_button.config(state=tk.DISABLED)
        close_button.config(state=tk.DISABLED)
        try:
            initial_position_set = True

            # Reset the motor position to 0
            current_position = 0
            current_speed = 0

            # Update position and speed labels in GUI
            root.after(0, update_position_label, current_position)
            root.after(0, update_speed_label, current_speed)
            root.after(0, update_initial_position_label, 0)

            print(f"Resetting motor to position 0...")
            send_ascii_command(motor_serial, "S.1=250")  # Set motor speed to 0
            send_ascii_command(motor_serial, "P.1=0")  # Move motor to position 0
            send_ascii_command(motor_serial, "^.1")  # Start motor movement

            root.after(3000, lambda: move_motor_button.config(state=tk.NORMAL))
            root.after(3000, lambda: reset_motor_button.config(state=tk.NORMAL))
            root.after(3000, lambda: close_button.config(state=tk.NORMAL))
            root.after(3000, lambda: status_label.config(text="Motor Ready", foreground="green"))


        except Exception as e:
            print(f"Error resetting motor position: {e}")
            status_label.config(text="Error resetting motor", foreground="red")
            root.after(3000, lambda: move_motor_button.config(state=tk.NORMAL))
            root.after(3000, lambda: reset_motor_button.config(state=tk.NORMAL))


def update_position_label(position):
    position_label_value.config(text=f"Position: {position}")

def update_speed_label(speed):
    speed_label_value.config(text=f"Speed: {speed}")

def update_initial_position_label(position):
    # Only update initial position if it was moved from position 0
    if position != 0:
        initial_position_label_value.config(text=f"Initial Position: {position}")
    else:
        initial_position_label_value.config(text=f"Initial Position: 0")

def update_motor_frequency():
    """
    Update the motor movement frequency (moves per minute) every minute.
    """
    global motor_move_count, last_frequency_update_time,total_motor_moves

    current_time = time.time()

    if current_time - last_frequency_update_time >= 60:  # Update every minute (60 seconds)
        root.after(0, update_frequency_label, motor_move_count)  # Update frequency label in GUI
        motor_move_count = 0  # Reset move count
        last_frequency_update_time = current_time  # Reset timestamp

    root.after(60000, update_motor_frequency)  # Schedule function to run again after 60 seconds


# Function to update the frequency label
def update_frequency_label(frequency):
    frequency_label_value.config(text=f"{frequency} moves/min")

def update_total_moves_label(total_moves):
    total_moves_label_value.config(text=f"Total Moves: {total_moves}")


# GUI Setup
root = tk.Tk()
root.title("Motor Control")
root.geometry("700x600")

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

# Motor control frame
motor = tk.Frame(root)
motor.pack(pady=20)


motor_container = tk.LabelFrame(motor, text="Motor Control", padx=10, pady=10)
motor_container.pack(padx=10)

# Button to move motor
move_motor_button = tk.Button(motor_container, text="Move Motor", command=move_motor_randomly, font=("Arial", 12),state=tk.DISABLED)
move_motor_button.pack(side="top", padx=10)

# Reset button to set motor position to 0
reset_motor_button = tk.Button(motor_container, text="Reset Motor", command=reset_motor_position, font=("Arial", 12),state=tk.DISABLED)
reset_motor_button.pack(side="top", padx=10)

position_label_value = tk.Label(motor_container, text="Position: 0", font=("Arial", 14))
position_label_value.pack(pady=5)

initial_position_label_value = tk.Label(motor_container, text="Initial Position: None", font=("Arial", 14))
initial_position_label_value.pack(pady=5)

speed_label_value = tk.Label(motor_container, text="Speed: 0", font=("Arial", 14))
speed_label_value.pack(pady=5)

frequency_label_value = tk.Label(motor_container, text="0 moves/min", font=("Arial", 14))
frequency_label_value.pack(pady=5)

total_moves_label_value = tk.Label(motor_container, text="Total Moves: 0", font=("Arial", 14))
total_moves_label_value.pack(pady=5)


root.mainloop()
