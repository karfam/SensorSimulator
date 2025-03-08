import time
import threading
import tkinter as tk
from tkinter import Canvas
from PIL import Image, ImageTk
from pymavlink import mavutil

# Connect to MAVProxy's input port
mp_target = 'udp:127.0.0.1:14550'

print("Connecting to MAVProxy...")
 # Pause for 3 seconds before connecting

master_30 = mavutil.mavlink_connection(mp_target, source_system=10, source_component=30)
time.sleep(3) 
master_31 = mavutil.mavlink_connection(mp_target, source_system=10, source_component=31)

print(f"Connected to MAVProxy at {mp_target} as system ID 10")

# Define the initial parameters for the PARAM_VALUE message
param_value_30 = 400.0  # Default CO2 value for comp 30 in ppm
param_value_31 = 400.0  # Default CO2 value for comp 31 in ppm
param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
param_count = 1
param_index = 0
send_frequency_hz_30 = 1  # Default frequency in Hz for comp 30
send_frequency_hz_31 = 1  # Default frequency in Hz for comp 31

# Wait for heartbeat
print("Waiting for heartbeat from MAVProxy/SITL...")
master_30.wait_heartbeat()
master_31.wait_heartbeat()
print("Heartbeat received! Starting message transmission.")

# Function to send heartbeat
def send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

# Function to send PARAM_VALUE message
def send_param_value(master, param_id, value):
    msg = master.mav.param_value_encode(
        param_id.encode('ascii'),
        value,
        param_type,
        param_count,
        param_index
    )
    msg.target_system = 1
    msg.target_component = 1
    master.mav.send(msg)
    print(f"PARAM_VALUE message sent: {param_id} = {value}")

# Thread function for periodic message sending
def message_sender_thread(master, param_id, get_value, get_frequency):
    while True:
        send_heartbeat(master)
        send_param_value(master, param_id, get_value())
        time.sleep(1 / get_frequency())

# Start the message sender threads
threading.Thread(target=message_sender_thread, args=(master_30, "CO2", lambda: param_value_30, lambda: send_frequency_hz_30), daemon=True).start()
threading.Thread(target=message_sender_thread, args=(master_31, "HDCO2", lambda: param_value_31, lambda: send_frequency_hz_31), daemon=True).start()

# GUI for controlling parameter values
root = tk.Tk()
root.title("MAVLink PARAM_VALUE GUI")
root.configure(bg="white")

# Add a background logo
def add_background_logo():
    canvas = Canvas(root, width=600, height=400, highlightthickness=0)
    canvas.pack(fill="both", expand=True)

    # Load and display the image
    try:
        image = Image.open("ieraxlogo.png")  # Replace with your logo file
        image = image.resize((600, 400), Image.ANTIALIAS)
        bg_image = ImageTk.PhotoImage(image)
        canvas.create_image(0, 0, image=bg_image, anchor="nw")
        return canvas, bg_image
    except FileNotFoundError:
        print("Logo file not found. Ensure 'ieraxlogo.png' is in the same directory.")
        return None, None

canvas, bg_image = add_background_logo()

frame = tk.Frame(root, bg="white")
frame.place(relwidth=0.8, relheight=0.8, relx=0.1, rely=0.1)

def slider_update_value_30(val):
    global param_value_30
    param_value_30 = float(val)

def slider_update_value_31(val):
    global param_value_31
    param_value_31 = float(val)

def slider_update_hz_30(val):
    global send_frequency_hz_30
    send_frequency_hz_30 = float(val)

def slider_update_hz_31(val):
    global send_frequency_hz_31
    send_frequency_hz_31 = float(val)

# Sliders for component 30
tk.Label(frame, text="Set PARAM_VALUE (CO2 in ppm) for Comp 30", bg="white").pack()
param_slider_30 = tk.Scale(frame, from_=0.0, to=20000.0, resolution=1.0, orient=tk.HORIZONTAL, command=slider_update_value_30, length=400, bg="white", highlightthickness=0, troughcolor="white")
param_slider_30.set(param_value_30)
param_slider_30.pack()

tk.Label(frame, text="Set Frequency (Hz) for Comp 30", bg="white").pack()
hz_slider_30 = tk.Scale(frame, from_=1.0, to=10.0, resolution=0.1, orient=tk.HORIZONTAL, command=slider_update_hz_30, length=400, bg="white", highlightthickness=0, troughcolor="white")
hz_slider_30.set(send_frequency_hz_30)
hz_slider_30.pack()

# Sliders for component 31
tk.Label(frame, text="Set PARAM_VALUE (HDCO2 in ppm) for Comp 31", bg="white").pack()
param_slider_31 = tk.Scale(frame, from_=0.0, to=20000.0, resolution=1.0, orient=tk.HORIZONTAL, command=slider_update_value_31, length=400, bg="white", highlightthickness=0, troughcolor="white")
param_slider_31.set(param_value_31)
param_slider_31.pack()

tk.Label(frame, text="Set Frequency (Hz) for Comp 31", bg="white").pack()
hz_slider_31 = tk.Scale(frame, from_=1.0, to=10.0, resolution=0.1, orient=tk.HORIZONTAL, command=slider_update_hz_31, length=400, bg="white", highlightthickness=0, troughcolor="white")
hz_slider_31.set(send_frequency_hz_31)
hz_slider_31.pack()

# Start the Tkinter main loop
try:
    root.mainloop()
except KeyboardInterrupt:
    print("GUI closed, stopping program.")
