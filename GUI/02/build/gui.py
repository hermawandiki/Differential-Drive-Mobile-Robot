import serial
import time
import json
import re

# This file was generated by the Tkinter Designer by Parth Jadhav
# https://github.com/ParthJadhav/Tkinter-Designer


from pathlib import Path

# from tkinter import *
# Explicit imports to satisfy Flake8
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage


OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path(r"/home/pi/Downloads/GUI DDMR/02/build/assets/frame0")


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def blink_rectangle():
    current_color = canvas.itemcget(led, 'fill')
    new_color = "#FF0000" if current_color == "#FFFFFF" else "#FFFFFF"
    canvas.itemconfig(led, fill=new_color)
    window.after(100, blink_rectangle)  # Kedip setiap 200ms
#
def send_character(char):
    if arduino.is_open and char:
        try:
            arduino.write(char.encode())  # Kirim karakter
            print(f"Karakter '{char}' telah dikirim")
            canvas.itemconfig(led, fill="#59FF00")  # Ubah warna jadi hijau
            window.after(100, lambda: canvas.itemconfig(led, fill="#FFFFFF"))  # Ubah jadi merah setelah 200ms
        except Exception as e:
            print(f"Kesalahan: {e}")
            blink_rectangle()  # Mulai berkedip jika error
#
def update_entry(new_text):
    # Ambil teks yang sudah ada di entry_6
    current_text = entry_6.get("1.0", "end-1c").strip()
    
    # Cek apakah teks baru berbeda dari teks yang sudah ada
    if current_text != new_text:
        entry_6.delete("1.0", "end")  # Hapus teks yang ada
        entry_6.insert("end", new_text)  # Masukkan teks baru
#
def button_2_click():
    send_character('T') # teleoperation
    update_entry("Teleoperation")
#
def button_3_click():
    send_character('A') # autonomous
    update_entry("Autonomous")
#
def button_4_click():
    send_character('W') # wall following
    update_entry("Wall Following")
#
def button_5_click():
    send_character('P') # point to point
    update_entry("Point to Point")
#    
def on_button_7_press(event):
    send_character('I')
# 
def on_button_7_release(event):
    send_character('X')
#
def on_button_8_press(event):
    send_character('L')
#
def on_button_8_release(event):
    send_character('X')
#
def on_button_9_press(event):
    send_character('J')
#
def on_button_9_release(event):
    send_character('X')
#
def on_button_10_press(event):
    send_character('K')
#
def on_button_10_release(event):
    send_character('X')
#

window = Tk()
Tk.title(window, "GUI DDMR")

window.geometry("1000x600")
window.configure(bg = "#FFFFFF")


canvas = Canvas(
    window,
    bg = "#FFFFFF",
    height = 600,
    width = 1000,
    bd = 0,
    highlightthickness = 0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
canvas.create_rectangle(
    0.0,
    0.0,
    1000.0,
    600.0,
    fill="#EFEFEF",
    outline="")

image_image_1 = PhotoImage(
    file=relative_to_assets("image_1.png"))
image_1 = canvas.create_image(
    294.0,
    55.0,
    image=image_image_1
)

image_image_2 = PhotoImage(
    file=relative_to_assets("image_2.png"))
image_2 = canvas.create_image(
    155.0,
    208.0,
    image=image_image_2
)

image_image_3 = PhotoImage(
    file=relative_to_assets("image_3.png"))
image_3 = canvas.create_image(
    294.0,
    424.0,
    image=image_image_3
)

image_image_4 = PhotoImage(
    file=relative_to_assets("image_4.png"))
image_4 = canvas.create_image(
    433.0,
    208.0,
    image=image_image_4
)

image_image_5 = PhotoImage(
    file=relative_to_assets("image_5.png"))
image_5 = canvas.create_image(
    78.0,
    54.0,
    image=image_image_5
)

image_image_6 = PhotoImage(
    file=relative_to_assets("image_6.png"))
image_6 = canvas.create_image(
    671.0,
    58.0,
    image=image_image_6
)

button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: send_character('M'),
    # print("button_1 clicked"),
    relief="flat"
)
button_1.place(
    x=756.0,
    y=48.0,
    width=100.0,
    height=30.0
)

button_image_2 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_2 = Button(
    image=button_image_2,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_2_click(),
    # print("button_2 clicked"),
    relief="flat"
)
button_2.place(
    x=281.0,
    y=549.0,
    width=100.0,
    height=20.0
)

button_image_3 = PhotoImage(
    file=relative_to_assets("button_3.png"))
button_3 = Button(
    image=button_image_3,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_3_click(),
    # print("button_3 clicked"),
    relief="flat"
)
button_3.place(
    x=394.0,
    y=549.0,
    width=100.0,
    height=20.0
)

button_image_4 = PhotoImage(
    file=relative_to_assets("button_4.png"))
button_4 = Button(
    image=button_image_4,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_4_click(),
    # print("button_4 clicked"),
    relief="flat"
)
button_4.place(
    x=507.0,
    y=549.0,
    width=100.0,
    height=20.0
)

button_image_5 = PhotoImage(
    file=relative_to_assets("button_5.png"))
button_5 = Button(
    image=button_image_5,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_5_click(),
    # print("button_5 clicked"),
    relief="flat"
)
button_5.place(
    x=620.0,
    y=549.0,
    width=100.0,
    height=20.0
)

button_image_6 = PhotoImage(
    file=relative_to_assets("button_6.png"))
button_6 = Button(
    image=button_image_6,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: send_character('N'),
    # print("button_6 clicked"),
    relief="flat"
)
button_6.place(
    x=866.0,
    y=48.0,
    width=100.0,
    height=30.0
)

image_image_7 = PhotoImage(
    file=relative_to_assets("image_7.png"))
image_7 = canvas.create_image(
    771.0,
    424.0,
    image=image_image_7
)

button_image_7 = PhotoImage(
    file=relative_to_assets("button_7.png"))
button_7 = Button(
    image=button_image_7,
    borderwidth=0,
    highlightthickness=0,
    # command=lambda: send_character('W'),
    # print("button_7 clicked"),
    relief="flat"
)
button_7.place(
    x=638.0,
    y=140.0,
    width=60.0,
    height=60.0
)
button_7.bind('<ButtonPress>', on_button_7_press)   # Rising edge
button_7.bind('<ButtonRelease>', on_button_7_release)  # Falling edge

button_image_8 = PhotoImage(
    file=relative_to_assets("button_8.png"))
button_8 = Button(
    image=button_image_8,
    borderwidth=0,
    highlightthickness=0,
    # command=lambda: send_character('D'),
    # print("button_8 clicked"),
    relief="flat"
)
button_8.place(
    x=694.0,
    y=190.0,
    width=60.0,
    height=60.0
)
button_8.bind('<ButtonPress>', on_button_8_press)   # Rising edge
button_8.bind('<ButtonRelease>', on_button_8_release)  # Falling edge

button_image_9 = PhotoImage(
    file=relative_to_assets("button_9.png"))
button_9 = Button(
    image=button_image_9,
    borderwidth=0,
    highlightthickness=0,
    # command=lambda: send_character('A'),
    # print("button_9 clicked"),
    relief="flat"
)
button_9.place(
    x=582.0,
    y=190.0,
    width=60.0,
    height=60.0
)
button_9.bind('<ButtonPress>', on_button_9_press)   # Rising edge
button_9.bind('<ButtonRelease>', on_button_9_release)  # Falling edge

button_image_10 = PhotoImage(
    file=relative_to_assets("button_10.png"))
button_10 = Button(
    image=button_image_10,
    borderwidth=0,
    highlightthickness=0,
    # command=lambda: send_character('S'),
    # print("button_10 clicked"),
    relief="flat"
)
button_10.place(
    x=638.0,
    y=240.0,
    width=60.0,
    height=60.0
)
button_10.bind('<ButtonPress>', on_button_10_press)   # Rising edge
button_10.bind('<ButtonRelease>', on_button_10_release)  # Falling edge

image_image_8 = PhotoImage(
    file=relative_to_assets("image_8.png"))
image_8 = canvas.create_image(
    874.0,
    208.0,
    image=image_image_8
)

entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    155.0,
    199.0,
    image=entry_image_1
)
entry_1 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_1.place(
    x=63.0,
    y=187.0,
    width=184.0,
    height=22.0
)

entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    928.0,
    267.0,
    image=entry_image_2
)
entry_2 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 10)
)
entry_2.place(
    x=919.0,
    y=258.0,
    width=18.0,
    height=16.0
)

entry_image_3 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_3 = canvas.create_image(
    848.5,
    267.0,
    image=entry_image_3
)
entry_3 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 10)
)
entry_3.place(
    x=835.0,
    y=255.0,
    width=45.0,
    height=23.0
)

entry_image_4 = PhotoImage(
    file=relative_to_assets("entry_4.png"))
entry_bg_4 = canvas.create_image(
    848.5,
    176.0,
    image=entry_image_4
)
entry_4 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 10)
)
entry_4.place(
    x=835.0,
    y=165.0,
    width=45.0,
    height=23.0
)

entry_image_5 = PhotoImage(
    file=relative_to_assets("entry_5.png"))
entry_bg_5 = canvas.create_image(
    928.5,
    176.0,
    image=entry_image_5
)
entry_5 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 10)
)
entry_5.place(
    x=915.0,
    y=167.0,
    width=27.0,
    height=16.0
)

entry_image_6 = PhotoImage(
    file=relative_to_assets("entry_6.png"))
entry_bg_6 = canvas.create_image(
    649.5,
    64.0,
    image=entry_image_6
)
entry_6 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 10)

)
entry_6.place(
    x=601.0,
    y=55.0,
    width=97.0,
    height=16.0
)

entry_image_7 = PhotoImage(
    file=relative_to_assets("entry_7.png"))
entry_bg_7 = canvas.create_image(
    848.5,
    221.0,
    image=entry_image_7
)
entry_7 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 10)
)
entry_7.place(
    x=835.0,
    y=210.0,
    width=45.0,
    height=23.0
)

entry_image_8 = PhotoImage(
    file=relative_to_assets("entry_8.png"))
entry_bg_8 = canvas.create_image(
    928.5,
    221.0,
    image=entry_image_8
)
entry_8 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 10)
)
entry_8.place(
    x=915.0,
    y=212.0,
    width=27.0,
    height=16.0
)

entry_image_9 = PhotoImage(
    file=relative_to_assets("entry_9.png"))
entry_bg_9 = canvas.create_image(
    155.0,
    269.0,
    image=entry_image_9
)
entry_9 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_9.place(
    x=63.0,
    y=257.0,
    width=184.0,
    height=22.0
)

entry_image_10 = PhotoImage(
    file=relative_to_assets("entry_10.png"))
entry_bg_10 = canvas.create_image(
    97.68065643310547,
    407.0,
    image=entry_image_10
)
entry_10 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_10.place(
    x=80,
    y=390.0,
    width=50,
    height=25.0
)

entry_image_11 = PhotoImage(
    file=relative_to_assets("entry_11.png"))
entry_bg_11 = canvas.create_image(
    97.68065643310547,
    486.0,
    image=entry_image_11
)
entry_11 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_11.place(
    x=80,
    y=470.0,
    width=50,
    height=25.0
)

entry_image_12 = PhotoImage(
    file=relative_to_assets("entry_12.png"))
entry_bg_12 = canvas.create_image(
    227.68062591552734,
    486.0,
    image=entry_image_12
)
entry_12 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_12.place(
    x=210,
    y=470.0,
    width=50,
    height=25.0
)

entry_image_13 = PhotoImage(
    file=relative_to_assets("entry_13.png"))
entry_bg_13 = canvas.create_image(
    357.68062591552734,
    485.0,
    image=entry_image_13
)
entry_13 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_13.place(
    x=340,
    y=470.0,
    width=50,
    height=25.0
)

entry_image_14 = PhotoImage(
    file=relative_to_assets("entry_14.png"))
entry_bg_14 = canvas.create_image(
    227.68062591552734,
    407.0,
    image=entry_image_14
)
entry_14 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_14.place(
    x=210,
    y=390.0,
    width=50,
    height=25.0
)

entry_image_15 = PhotoImage(
    file=relative_to_assets("entry_15.png"))
entry_bg_15 = canvas.create_image(
    357.68062591552734,
    407.0,
    image=entry_image_15
)
entry_15 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_15.place(
    x=340,
    y=390.0,
    width=50,
    height=25.0
)

entry_image_16 = PhotoImage(
    file=relative_to_assets("entry_16.png"))
entry_bg_16 = canvas.create_image(
    487.68062591552734,
    406.0,
    image=entry_image_16
)
entry_16 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_16.place(
    x=460,
    y=390.0,
    width=60,
    height=25.0
)

entry_image_17 = PhotoImage(
    file=relative_to_assets("entry_17.png"))
entry_bg_17 = canvas.create_image(
    487.68062591552734,
    485.0,
    image=entry_image_17
)
entry_17 = Text(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    font=("Arial ", 15)
)
entry_17.place(
    x=460,
    y=470.0,
    width=60,
    height=25.0
)

entry_image_18 = PhotoImage(
    file=relative_to_assets("entry_18.png"))
entry_bg_18 = canvas.create_image(
    640.6806564331055,
    407.0,
    image=entry_image_18
)
entry_18 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_18.place(
    x=581.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_19 = PhotoImage(
    file=relative_to_assets("entry_19.png"))
entry_bg_19 = canvas.create_image(
    640.6806564331055,
    485.0,
    image=entry_image_19
)
entry_19 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_19.place(
    x=581.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_20 = PhotoImage(
    file=relative_to_assets("entry_20.png"))
entry_bg_20 = canvas.create_image(
    767.6806564331055,
    407.0,
    image=entry_image_20
)
entry_20 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_20.place(
    x=708.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_21 = PhotoImage(
    file=relative_to_assets("entry_21.png"))
entry_bg_21 = canvas.create_image(
    767.6806564331055,
    485.0,
    image=entry_image_21
)
entry_21 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_21.place(
    x=708.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_22 = PhotoImage(
    file=relative_to_assets("entry_22.png"))
entry_bg_22 = canvas.create_image(
    894.6806564331055,
    407.0,
    image=entry_image_22
)
entry_22 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_22.place(
    x=835.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_23 = PhotoImage(
    file=relative_to_assets("entry_23.png"))
entry_bg_23 = canvas.create_image(
    894.6806564331055,
    485.0,
    image=entry_image_23
)
entry_23 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_23.place(
    x=835.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_24 = PhotoImage(
    file=relative_to_assets("entry_24.png"))
entry_bg_24 = canvas.create_image(
    433.0,
    199.0,
    image=entry_image_24
)
entry_24 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_24.place(
    x=341.0,
    y=187.0,
    width=184.0,
    height=22.0
)

entry_image_25 = PhotoImage(
    file=relative_to_assets("entry_25.png"))
entry_bg_25 = canvas.create_image(
    433.0,
    269.0,
    image=entry_image_25
)
entry_25 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_25.place(
    x=341.0,
    y=257.0,
    width=184.0,
    height=22.0
)

button_image_11 = PhotoImage(
    file=relative_to_assets("button_11.png"))
button_11 = Button(
    image=button_image_11,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: print("button_11 clicked"),
    relief="flat"
)
button_11.place(
    x=921.0,
    y=329.0,
    width=50.0,
    height=15.0
)

led = canvas.create_rectangle(
    946.0,
    21.0,
    966.0,
    41.0,
    fill="#FFFFFF",
    outline="")
window.resizable(False, False)

def setup():
    send_character('T') # teleoperation
    update_entry("Teleoperation")
# 

# def loop():
#     if arduino.in_waiting > 0:
#         data = arduino.readline().decode('utf-8').strip()
#         pattern = r"U(\d+)U(\d+)U(\d+)E(\d+)E(\d+)Y([-]?\d+\.\d+)P([-]?\d+\.\d+)R([-]?\d+\.\d+)X([-]?\d+\.\d+)Y([-]?\d+\.\d+)W([-]?\d+\.\d+)"
#         # Menggunakan re.match untuk mencocokkan pattern
#         match = re.match(pattern, data)

#         if match:
#             # Parsing hasilnya
#             us_kiri         = int(match.group(1))
#             us_depan        = int(match.group(2))
#             us_kanan        = int(match.group(3))
#             encoder_kiri    = int(match.group(4))
#             encoder_kanan   = int(match.group(5))
#             yaw             = float(match.group(6))
#             pitch           = float(match.group(7))
#             roll            = float(match.group(8))
#             pos_x           = float(match.group(9))
#             pos_y           = float(match.group(10))
#             pos_w           = float(match.group(11))
        
#             entry_13.delete("1.0", "end")
#             entry_13.insert("end", roll)
#             entry_11.delete("1.0", "end")
#             entry_11.insert("end", yaw)
#             entry_12.delete("1.0", "end")
#             entry_12.insert("end", pitch)

#             entry_14.delete("1.0", "end")
#             entry_14.insert("end", us_depan)
#             entry_15.delete("1.0", "end")
#             entry_15.insert("end", us_kanan)
#             entry_10.delete("1.0", "end")
#             entry_10.insert("end", us_kiri)

#             entry_16.delete("1.0", "end")
#             entry_16.insert("end", encoder_kiri)
#             entry_17.delete("1.0", "end")
#             entry_17.insert("end", encoder_kanan)

#             entry_3.delete("1.0", "end")
#             entry_3.insert("end", pos_w)
#             entry_4.delete("1.0", "end")
#             entry_4.insert("end", pos_x)
#             entry_7.delete("1.0", "end")
#             entry_7.insert("end", pos_y)

#     window.after(100, loop)

def loop():
    if arduino.in_waiting > 0:
        raw_data = arduino.readline().decode('utf-8').strip()
        # print(data)
        data_json = json.loads(raw_data)

        us_kiri         = data_json['us_kiri']
        us_depan        = data_json['us_depan']
        us_kanan        = data_json['us_kanan']
        yaw             = data_json['yaw']
        pitch           = data_json['pitch']
        roll            = data_json['roll']
        encoder_kiri    = data_json['encoder_l']
        encoder_kanan   = data_json['encoder_r']
        pos_x           = data_json['pos_x']
        pos_y           = data_json['pos_y']
        pos_w           = data_json['pos_w']

        entry_13.delete("1.0", "end")
        entry_13.insert("end", roll)
        entry_11.delete("1.0", "end")
        entry_11.insert("end", yaw)
        entry_12.delete("1.0", "end")
        entry_12.insert("end", pitch)

        entry_14.delete("1.0", "end")
        entry_14.insert("end", us_depan)
        entry_15.delete("1.0", "end")
        entry_15.insert("end", us_kanan)
        entry_10.delete("1.0", "end")
        entry_10.insert("end", us_kiri)

        entry_16.delete("1.0", "end")
        entry_16.insert("end", encoder_kiri)
        entry_17.delete("1.0", "end")
        entry_17.insert("end", encoder_kanan)

        entry_3.delete("1.0", "end")
        entry_3.insert("end", pos_w)
        entry_4.delete("1.0", "end")
        entry_4.insert("end", pos_x)
        entry_7.delete("1.0", "end")
        entry_7.insert("end", pos_y)
    
    window.after(100, loop)


if __name__ == "__main__":
    setup()
    loop()
    window.mainloop()
