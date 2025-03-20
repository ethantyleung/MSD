import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import signal
import sys
from matplotlib.widgets import Button

numPoints = 100  # Number of data points to display

show_distance =True
show_armHeight = True
show_accelValueRod = True
show_accelValueMass = True

# Function to get available COM ports
def get_com_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Get the renderer to parse the legend box coordinate
def find_renderer(fig):
    if hasattr(fig.canvas, "get_renderer"):
        # Some backends, such as TkAgg, have the get_renderer method, which 
        # makes this easy.
        renderer = fig.canvas.get_renderer()
    else:
        # Other backends do not have the get_renderer method, so we have a work 
        # around to find the renderer.  Print the figure to a temporary file 
        # object, and then grab the renderer that was used.
        import io
        fig.canvas.print_pdf(io.BytesIO())
        renderer = fig._cachedRenderer
    return(renderer)

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
        y_vals_accelValueRod = []
        y_vals_accelValueMass = []

        # Initialize previous distance
        prev_distance = None

        # Create figure and axis
        fig, ax = plt.subplots(figsize=(13,6))
        line_distance, = ax.plot([], [], 'r-', label='Output Displacement')  # Initialize an empty line plot for distance with red color
        line_armHeight, = ax.plot([], [], 'b-', label='Input Displacement')  # Initialize an empty line plot for arm height with blue color
        line_accelValueRod, = ax.plot([], [], 'g-', label='Rod Acceleration')  # Initialize an empty line plot for acceleration with green color
        line_accelValueMass, = ax.plot([], [], 'y-', label='Mass Acceleration')  # Initialize an empty line plot for acceleration with yellow color
        # Add legend
        legend = ax.legend(loc='lower left')

        # Fetch legend coordinates to align buttons
        fig.canvas.draw()
        renderer = find_renderer(fig)
        bbox = legend.get_window_extent(renderer)
        bbox_in_fig_coords = bbox.transformed(ax.transAxes.inverted())
        button_x_coord = bbox_in_fig_coords.x0

        def toggle_show_distance(event):
            global show_distance
            show_distance = not show_distance
            line_distance.set_visible(show_distance)
        
        def toggle_show_armHeight(event):
            global show_armHeight
            show_armHeight = not show_armHeight
            line_armHeight.set_visible(show_armHeight)
        
        def toggle_show_accelValueRod(event):
            global show_accelValueRod
            show_accelValueRod = not show_accelValueRod
            line_accelValueRod.set_visible(show_accelValueRod)
        
        def toggle_show_accelValueMass(event):
            global show_accelValueMass
            show_accelValueMass = not show_accelValueMass
            line_accelValueMass.set_visible(show_accelValueMass)

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
            nonlocal x_vals, y_vals_distance, y_vals_armHeight, prev_distance, y_vals_accelValueRod, y_vals_accelValueMass  # Declare variables as nonlocal
            
            arduinoData_string = ""  # Initialize arduinoData_string
            
            try:
                # Read all available lines from serial
                while ser.in_waiting:
                    arduinoData_string = ser.readline().decode('utf-8').strip()
                
                # Process the most recent line
                if arduinoData_string:
                    try:
                        currentTime, distance, armHeight, accelValueRod, accelValueMass = map(float, arduinoData_string.split())
                        if -10 < distance < 20 and (prev_distance is None or abs(distance - prev_distance) <= 5):  # Check conditions
                            x_vals.append(currentTime)
                            y_vals_distance.append(distance)
                            y_vals_armHeight.append(armHeight)
                            y_vals_accelValueRod.append(accelValueRod)
                            y_vals_accelValueMass.append(accelValueMass)
                            prev_distance = distance  # Update previous distance

                            # Keep only the last 100 data points
                            x_vals = x_vals[-numPoints:]
                            y_vals_distance = y_vals_distance[-numPoints:]
                            y_vals_armHeight = y_vals_armHeight[-numPoints:]
                            y_vals_accelValueRod = y_vals_accelValueRod[-numPoints:]
                            y_vals_accelValueMass = y_vals_accelValueMass[-numPoints:]

                            # Update line data
                            line_distance.set_data(x_vals, y_vals_distance)
                            line_armHeight.set_data(x_vals, y_vals_armHeight)
                            line_accelValueRod.set_data(x_vals, y_vals_accelValueRod)
                            line_accelValueMass.set_data(x_vals, y_vals_accelValueMass)

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
                y_vals_accelValueRod = []
                y_vals_accelValueMass = []

            return line_distance, line_armHeight, line_accelValueRod , line_accelValueMass

        # Enable frame caching with a specified save_count
        ani = animation.FuncAnimation(fig, animate, interval=10, blit=False, save_count=numPoints)

        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Live Plot of Distance and Arm Height and Acceleration')

        # Defining buttons and adding their functionality
        # Red Button (Distance)
        distance_axes = fig.add_axes([button_x_coord,0.205,0.06,0.023])
        bred = Button(distance_axes, 'Show/Hide',color="red")
        bred.on_clicked(toggle_show_distance)
        # Blue Button (Arm Height)
        armHeight_axes = fig.add_axes([button_x_coord,0.1775,0.06,0.023])
        bblue = Button(armHeight_axes, 'Show/Hide',color="lightsteelblue")
        bblue.on_clicked(toggle_show_armHeight)
        # Yellow Button (Rod Z-Acceleration)
        accelValueRod_axes = fig.add_axes([button_x_coord,0.151,0.06,0.023])
        byellow = Button(accelValueRod_axes, 'Show/Hide',color="mediumseagreen")
        byellow.on_clicked(toggle_show_accelValueRod)
        # Green Button (Mass Z-Acceleration)
        accelValueMass_axes = fig.add_axes([button_x_coord,0.125,0.06,0.023])
        bgreen = Button(accelValueMass_axes, 'Show/Hide',color="yellow")
        bgreen.on_clicked(toggle_show_accelValueMass)
        
        # Switch to the TkAgg backend & Set the default window size to MAX
        plt.switch_backend('TkAgg')
        mng = plt.get_current_fig_manager()
        mng.window.state('zoomed')
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