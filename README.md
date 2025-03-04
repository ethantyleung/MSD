# MSD
Code for the Mass Spring Damper system designed by the UW Engineering Ideas Clinic for the ECE198 course.

### To run the Spring-Mass Damper Demo
1. Clone this repository
2. Open the msd_main.ino sketch in the Arduino IDE (click YES to any popups)
3. Connect your laptop to the Arduino
4. Connect a 12V power source to the Motor H Bridge as well
4. In the Arduino IDE, select the correct COM port and then upload the code to the Arduino
5. At this point the motor should begin moving 

### To run the serial_plotter.py script on a Windows system:
1. Install Python if you haven't already
2. Open up a Windows PowerShell terminal as administrator
3. Install the serial & matplotlib module by running 'py -m pip install pyserial matplotlib' in the terminal
4. Navigate to the folder that your serial_plotter.py file is in
5. Run 'py .\serial_plotter.py' in terminal to start the script
6. Match the COM port to the one listed in the Arduino IDE
7. Click start! A GUI should pop up and the graph should begin plotting.