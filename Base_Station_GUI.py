import tkinter as tk
import serial
import threading
import time
import customtkinter

try:
    ser = serial.Serial('COM15', 9600)
except serial.SerialException:
    ser = None
    print("Serial port not found or in use.")

send_interval = 0.25  # 100 ms

def send_serial_data():
    if ser:
        propeller_speed = powerSlider.get()
        rudder_angle = rudderSlider.get()
        enable_val = enable_var.get()
        address_val = int(address_spinbox.get())

        mode = 0
        direction = 1 if propeller_speed >= 0 else 0
        byte2 = (direction & 0x01) | (enable_val << 1)
        byte3 = abs(int(propeller_speed))
        byte4 = int(rudder_angle)

        serial_data = f"{address_val},{mode},{byte2},{byte3},{byte4}\n".encode()
        ser.write(serial_data)
        display_text(f"Sent: {serial_data.decode().strip()}")
    app.after(int(send_interval * 1000), send_serial_data)  # Schedule next send

def update_values():
    propeller_speed = powerSlider.get()
    rudder_angle = rudderSlider.get()
    enable_val = enable_var.get()
    address_val = int(address_spinbox.get())
    label1.config(text=f"Propeller Speed: {propeller_speed}%")
    label2.config(text=f"Rudder Angle: {rudder_angle}°, Enable: {enable_val}, Address: {address_val}")

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

def on_closing():
    
    if ser:
        final_data = f"{int(address_spinbox.get())},0,0,0,90\n".encode() #disable and zero speed before closing.
        ser.write(final_data)
        ser.close()
    app.destroy()

app = customtkinter.CTk()


# root = tk.Tk()
app.title("Tkinter Serial GUI")
app.geometry("600x400")
app.protocol("WM_DELETE_WINDOW", on_closing) #call on_closing on exit.

powerSlider = customtkinter.CTkSlider(app, from_=0, to=100, command=lambda x: update_values(), orientation=customtkinter.VERTICAL)
powerSlider.grid(row=0, column=0, padx=20, pady=20)
powerSlider.set(0)

rudderSlider = customtkinter.CTkSlider(app, from_=0, to=100, command=lambda x: update_values())
rudderSlider.grid(row=0, column=3, padx=20, pady=20)


enable_var = customtkinter.IntVar()
enable_switch = customtkinter.CTkSwitch(app, text="Enable Motor", variable=enable_var, command=update_values)
enable_switch.grid(row=1, column=0, padx=20, pady=20)


# Vertical Slider (Propeller Speed)
# slider1 = tk.Scale(app, from_=100, to=-100, orient=tk.VERTICAL, command=lambda x: update_values())
# slider1.set(0)
label1 = tk.Label(app, text="Propeller Speed: 0%")

# Middle Frame for Switch, Labels, and Address
middle_frame = tk.Frame(app)

label2 = tk.Label(middle_frame, text="Rudder Angle: 90°")
label2.pack(pady=5)

# tk.Checkbutton(middle_frame, text="Enable", variable=enable_var, command=update_values)
# enable_switch.pack(pady=5)

address_spinbox = tk.Spinbox(middle_frame, from_=1, to=255, width=5)
address_spinbox.pack(pady=5)

# Horizontal Slider (Rudder Angle)
slider2 = tk.Scale(app, from_=45, to=135, orient=tk.HORIZONTAL, command=lambda x: update_values())
slider2.set(90)

# Text Box for Serial Data
text_box = tk.Text(app, height=10, width=50, state=tk.DISABLED)
# text_box.grid(row=2, column=0, columnspan=3, padx=10, pady=10)

# Layout using grid
# label1.grid(row=0, column=0, sticky="ns", padx=10, pady=10)
# slider1.grid(row=1, column=0, sticky="ns", padx=10, pady=10)
# middle_frame.grid(row=1, column=1, padx=10, pady=10)
# slider2.grid(row=1, column=2, sticky="ew", padx=10, pady=10)

# Configure grid weights to allow horizontal slider to expand
# app.grid_columnconfigure(2, weight=1)

if ser:
    serial_thread = threading.Thread(target=read_serial)
    serial_thread.daemon = True
    serial_thread.start()
    app.after(int(send_interval * 1000), send_serial_data) #start the periodic send.

# root.mainloop()
app.mainloop()