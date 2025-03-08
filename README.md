MAVLink Sensor Simulator GUI
This repository contains a Python script that connects to MAVProxy and sends MAVLink PARAM_VALUE messages that emulate a  CO2 and a HD CO2 parameter. 
The script includes a GUI for controlling the parameter values and the frequency of the messages.

Features
Connects to MAVProxy via UDP.
Sends PARAM_VALUE messages for two sensors.
GUI for controlling parameter values and message frequency.

Installation

pip install pymavlink pillow
Ensure MAVProxy is running and listening on the specified UDP port (default: 127.0.0.1:14550).
Run the script:

python mavlink_param_value_gui.py

Use the GUI to adjust the CO2 and HDCO2 parameter values and the frequency of the messages.

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
pymavlink
tkinter
Pillow


![image](https://github.com/user-attachments/assets/55cb49f6-9809-4387-8570-df651fc234fa)
