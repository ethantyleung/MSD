import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import signal
import sys

numPoints = 100  # Number of data points to display

# Function to get available COM ports
def get_com_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Function to start the plot
def start_plot(port):
    try:
        # Initialize serial connection
        ser = serial.Serial(port, 115200)
        time.sleep(2)  # Wait for the serial connection to initialize

        # Initialize lists to store data
        x_vals = []
        y_vals_distance = []
        y_vals_armHeight = []
        y_vals_accelValue = []

        # Initialize previous distance
        prev_distance = None

        # Create figure and axis
        fig, ax = plt.subplots()
        line_distance, = ax.plot([], [], 'r-', label='Output Displacement')  # Initialize an empty line plot for distance with red color
        line_armHeight, = ax.plot([], [], 'b-', label='Input Displacement')  # Initialize an empty line plot for arm height with blue color
        line_accelValue, = ax.plot([], [], 'g-', label='Acceleration')  # Initialize an empty line plot for acceleration with green color

        # Add legend
        ax.legend()

        # Function to handle keyboard interrupt
        def signal_handler(sig, frame):
            print('Closing plot...')
            plt.close(fig)
            ser.close()
            sys.exit(0)

        # Register the signal handler
        signal.signal(signal.SIGINT, signal_handler)

        # Set up plot to call animate() function periodically
        def animate(i):
            nonlocal x_vals, y_vals_distance, y_vals_armHeight, prev_distance, y_vals_accelValue  # Declare variables as nonlocal
            
            arduinoData_string = ""  # Initialize arduinoData_string
            
            try:
                # Read all available lines from serial
                while ser.in_waiting:
                    arduinoData_string = ser.readline().decode('utf-8').strip()
                
                # Process the most recent line
                if arduinoData_string:
                    try:
                        currentTime, distance, armHeight, accelValue = map(float, arduinoData_string.split())
                        if -10 < distance < 20 and (prev_distance is None or abs(distance - prev_distance) <= 5):  # Check conditions
                            x_vals.append(currentTime)
                            y_vals_distance.append(distance)
                            y_vals_armHeight.append(armHeight)
                            y_vals_accelValue.append(accelValue)
                            prev_distance = distance  # Update previous distance

                            # Keep only the last 100 data points
                            x_vals = x_vals[-numPoints:]
                            y_vals_distance = y_vals_distance[-numPoints:]
                            y_vals_armHeight = y_vals_armHeight[-numPoints:]
                            y_vals_accelValue = y_vals_accelValue[-numPoints:]
                            
                            # Update line data
                            line_distance.set_data(x_vals, y_vals_distance)
                            line_armHeight.set_data(x_vals, y_vals_armHeight)
                            line_accelValue.set_data(x_vals, y_vals_accelValue)
                            ax.relim()  # Recalculate limits
                            ax.autoscale_view()  # Autoscale the view
                    except ValueError:
                        print("Error processing data")  # Debugging statement
            except serial.SerialException:
                print("Serial connection lost. Attempting to reconnect...")
                ser.close()
                time.sleep(2)
                ser.open()
                print("Reconnected to serial port.") 
                x_vals = []
                y_vals_distance = []
                y_vals_armHeight = []
                y_vals_accelValue = []

            return line_distance, line_armHeight, line_accelValue

        # Enable frame caching with a specified save_count
        ani = animation.FuncAnimation(fig, animate, interval=10, blit=False, save_count=numPoints)

        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Live Plot of Distance and Arm Height and Acceleration')
        plt.show()

        # Close the serial connection when the plot window is closed
        ser.close()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

# Create the main window
root = tk.Tk()
root.title("Select COM Port")

# Create a label
label = tk.Label(root, text="Select the COM port for the Arduino:")
label.pack(pady=10)

# Create a combobox for COM port selection
com_ports = get_com_ports()
com_port_var = tk.StringVar()
com_port_combobox = ttk.Combobox(root, textvariable=com_port_var, values=com_ports)
com_port_combobox.pack(pady=10)

# Create a button to start the plot
start_button = tk.Button(root, text="Start Plot", command=lambda: start_plot(com_port_var.get()))
start_button.pack(pady=10)

# Run the main loop
root.mainloop()