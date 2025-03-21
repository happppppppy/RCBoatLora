import tkinter as tk
import serial
import threading
import time

try:
    ser = serial.Serial('COM15', 9600)
except serial.SerialException:
    ser = None
    print("Serial port not found or in use.")

last_send_time = 0
send_interval = 0.1  # 10 Hz (1/10 = 0.1 seconds)

def update_values():
    global last_send_time
    propeller_speed = slider1.get()
    rudder_angle = slider2.get()
    enable_val = enable_var.get()
    address_val = int(address_spinbox.get())  # Get address as integer

    label1.config(text=f"Propeller Speed: {propeller_speed}%")
    label2.config(text=f"Rudder Angle: {rudder_angle}°, Enable: {enable_val}, Address: {address_val}")

    current_time = time.time()
    if ser and (current_time - last_send_time) >= send_interval:
        mode = 0  # Mode is always 0
        direction = 1 if propeller_speed >= 0 else 0  # 1 for forward, 0 for reverse
        byte2 = (direction & 0x01) | (enable_val << 1) # combine enable and direction into one byte.
        byte3 = int(abs(propeller_speed))
        byte4 = int(rudder_angle)

        serial_data = f"{address_val},{mode},{byte2},{byte3},{byte4}\n".encode() #format serial data as comma separated integers.
        ser.write(serial_data)
        display_text(f"Sent: {serial_data.decode().strip()}") #display sent data
        last_send_time = current_time

def display_text(text):
    text_box.config(state=tk.NORMAL)
    text_box.insert(tk.END, text + "\n")
    text_box.see(tk.END)
    text_box.config(state=tk.DISABLED)

def read_serial():
    if ser:
        while True:
            try:
                line = ser.readline().decode().strip()
                if line:
                    display_text(f"Received: {line}")
            except serial.SerialException:
                display_text("Serial port disconnected")
                break
            except UnicodeDecodeError:
                display_text("Unicode Decode Error")

root = tk.Tk()
root.title("Tkinter Serial GUI")

# Vertical Slider (Propeller Speed)
slider1 = tk.Scale(root, from_=100, to=-100, orient=tk.VERTICAL, command=lambda x: update_values())
slider1.set(0)
label1 = tk.Label(root, text="Propeller Speed: 0%")

# Middle Frame for Switch, Labels, and Address
middle_frame = tk.Frame(root)

label2 = tk.Label(middle_frame, text="Rudder Angle: 90°")
label2.pack(pady=5)

enable_var = tk.IntVar()
enable_switch = tk.Checkbutton(middle_frame, text="Enable", variable=enable_var, command=update_values)
enable_switch.pack(pady=5)

address_spinbox = tk.Spinbox(middle_frame, from_=1, to=255, width=5)
address_spinbox.pack(pady=5)

# Horizontal Slider (Rudder Angle)
slider2 = tk.Scale(root, from_=45, to=135, orient=tk.HORIZONTAL, command=lambda x: update_values())
slider2.set(90)

# Text Box for Serial Data
text_box = tk.Text(root, height=10, width=50, state=tk.DISABLED)
text_box.grid(row=2, column=0, columnspan=3, padx=10, pady=10)

# Layout using grid
label1.grid(row=0, column=0, sticky="ns", padx=10, pady=10)
slider1.grid(row=1, column=0, sticky="ns", padx=10, pady=10)
middle_frame.grid(row=1, column=1, padx=10, pady=10)
slider2.grid(row=1, column=2, sticky="ew", padx=10, pady=10)

# Configure grid weights to allow horizontal slider to expand
root.grid_columnconfigure(2, weight=1)

if ser:
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()

root.mainloop()

if ser:
    ser.close()