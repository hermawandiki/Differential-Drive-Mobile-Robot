from pathlib import Path
from tkinter import Tk, Canvas, Entry, Text, Button, PhotoImage
from socketio import Client
import threading
import json

OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path(r"D:\GitHub Data\Differential-Drive-Mobile-Robot\GUI\04\build\assets\frame0")

def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

class SensorData:
    def __init__(self):
        # Inisialisasi variabel sensor
        self.usl = 0
        self.usf = 0
        self.usr = 0
        self.encl = 0
        self.encr = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.kpl = 0
        self.kpr = 0
        self.kil = 0
        self.kir = 0
        self.kdl = 0
        self.kdr = 0
    
    def parsing_data_json(self, data):
        parsed_data = json.loads(data)
        self.usl = parsed_data['usl']
        self.usf = parsed_data['usf']
        self.usr = parsed_data['usr']
        self.encl = parsed_data['encl']
        self.encr = parsed_data['encr']
        self.pitch = parsed_data['pitch']
        self.roll = parsed_data['roll']
        self.yaw = parsed_data['yaw']
        self.kpl = parsed_data['kpl']
        self.kpr = parsed_data['kpr']
        self.kil = parsed_data['kil']
        self.kir = parsed_data['kir']
        self.kdl = parsed_data['kdl']
        self.kdr = parsed_data['kdr']

    def get_pid_data(self):
        return (self.kpl, self.kpr, self.kil, self.kir, self.kdl, self.kdr)

sensor_data = SensorData()

SERVER_URL = 'http://192.168.137.161:5000' 
sio = Client()

@sio.event
def connect():
    print('connection established')
    init_dashboard()

@sio.event
def disconnect():
    print('disconnected from server')

@sio.event
def msg(data):
    try:
        sensor_data.parsing_data_json(data)
        print('message received : ', data)
    except json.JSONDecodeError:
        print('Failed to decode JSON data')

def send_to_server(data):
    sio.emit('msg', data)

def set_data_pid():
    kdr = float(entry_1.get())
    kir = float(entry_2.get())
    kpr = float(entry_3.get())
    kdl = float(entry_4.get())
    kil = float(entry_5.get())
    kpl = float(entry_6.get())
    data = f"#{kpl},{kpr},{kil},{kir},{kdl},{kdr}\n"
    send_to_server(data)

def get_data_pid():
    kpl, kpr, kil, kir, kdl, kdr = sensor_data.get_pid_data()
    entry_1.delete(0, 'end')
    entry_2.delete(0, 'end')
    entry_3.delete(0, 'end')
    entry_4.delete(0, 'end')
    entry_5.delete(0, 'end')
    entry_6.delete(0, 'end')
    entry_1.insert(0, kdr)
    entry_2.insert(0, kir)
    entry_3.insert(0, kpr)
    entry_4.insert(0, kdl)
    entry_5.insert(0, kil)
    entry_6.insert(0, kpl)

def init_dashboard():
    entry_1.delete(0, 'end')
    entry_2.delete(0, 'end')
    entry_3.delete(0, 'end')
    entry_4.delete(0, 'end')
    entry_5.delete(0, 'end')
    entry_6.delete(0, 'end') 
    entry_7.delete(0, 'end')
    entry_8.delete(0, 'end')
    entry_9.delete(0, 'end')
    entry_10.delete(0, 'end')
    get_data_pid()
    entry_7.insert(0, 100)
    entry_8.insert(0, 100)
    entry_9.insert(0, 180)
    entry_10.insert(0, 300)

def update_dashboard():
    canvas.itemconfig(usl_text, text=sensor_data.usl)
    canvas.itemconfig(usf_text, text=sensor_data.usf)
    canvas.itemconfig(usr_text, text=sensor_data.usr)
    canvas.itemconfig(encl_text, text=sensor_data.encl)
    canvas.itemconfig(encr_text, text=sensor_data.encr)
    canvas.itemconfig(pitch_text, text=sensor_data.pitch)
    canvas.itemconfig(roll_text, text=sensor_data.roll)
    canvas.itemconfig(yaw_text, text=sensor_data.yaw)
    window.after(1, update_dashboard)

def button_1_clicked():
    send_to_server("P")
    canvas.itemconfig(current_mode, text="Test Motor")
def button_3_clicked():
    send_to_server("A")
    canvas.itemconfig(current_mode, text="Autonomous")
def button_4_clicked():
    send_to_server("T")
    canvas.itemconfig(current_mode, text="Teleoperation")
def button_5_clicked():
    send_to_server("N")
def button_6_clicked():
    current_mode_text = canvas.itemcget(current_mode, 'text')
    if current_mode_text == "Test Motor":  
        pwm1 = entry_8.get()
        pwm2 = entry_7.get()
        data = f"Q{pwm1},{pwm2}\n"
        send_to_server(data + "M")
    else:
        send_to_server("M")
def button_7_pressed(event):
    send_to_server("L" + entry_9.get())
def button_7_released(event):
    send_to_server("X")
def button_8_pressed(event):
    send_to_server("J" + entry_9.get())
def button_8_released(event):
    send_to_server("X")
def button_9_pressed(event):
    send_to_server("K" + entry_10.get())
def button_9_released(event):
    send_to_server("X")
def button_10_pressed(event):
    send_to_server("I" + entry_10.get())
def button_10_released(event):
    send_to_server("X")


window = Tk()
Tk.title(window, "GUI DDMR - V1.4")

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

button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_1_clicked(),
    relief="flat"
)
button_1.place(
    x=620.0,
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
    command=lambda: button_3_clicked(),
    relief="flat"
)
button_3.place(
    x=450.0,
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
    command=lambda: button_4_clicked(),
    relief="flat"
)
button_4.place(
    x=280.0,
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
    command=lambda: button_5_clicked(),
    relief="flat"
)
button_5.place(
    x=866.0,
    y=48.0,
    width=100.0,
    height=30.0
)

button_image_6 = PhotoImage(
    file=relative_to_assets("button_6.png"))
button_6 = Button(
    image=button_image_6,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: button_6_clicked(),
)
button_6.place(
    x=756.0,
    y=48.0,
    width=100.0,
    height=30.0
)

button_image_7 = PhotoImage(
    file=relative_to_assets("button_7.png"))
button_7 = Button(
    image=button_image_7,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_7.place(
    x=825.7906951904297,
    y=182.5,
    width=87.20930480957031,
    height=75.0
)
button_7.bind('<ButtonPress>', button_7_pressed)
window.bind('<KeyPress-l>', button_7_pressed)
button_7.bind('<ButtonRelease>', button_7_released)
window.bind('<KeyRelease-l>', button_7_released)

button_image_8 = PhotoImage(
    file=relative_to_assets("button_8.png"))
button_8 = Button(
    image=button_image_8,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_8.place(
    x=663.0,
    y=182.5,
    width=87.20930480957031,
    height=75.0
)
button_8.bind('<ButtonPress>', button_8_pressed)
window.bind('<KeyPress-j>', button_8_pressed)
button_8.bind('<ButtonRelease>', button_8_released)
window.bind('<KeyRelease-j>', button_8_released)

button_image_9 = PhotoImage(
    file=relative_to_assets("button_9.png"))
button_9 = Button(
    image=button_image_9,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_9.place(
    x=744.3953094482422,
    y=245.0,
    width=87.20930480957031,
    height=75.0
)
button_9.bind('<ButtonPress>', button_9_pressed)
window.bind('<KeyPress-k>', button_9_pressed)
button_9.bind('<ButtonRelease>', button_9_released)
window.bind('<KeyRelease-k>', button_9_released)

button_image_10 = PhotoImage(
    file=relative_to_assets("button_10.png"))
button_10 = Button(
    image=button_image_10,
    borderwidth=0,
    highlightthickness=0,
    relief="flat"
)
button_10.place(
    x=744.3953857421875,
    y=120.0,
    width=87.20930480957031,
    height=75.0
)
button_10.bind('<ButtonPress>', button_10_pressed)
window.bind('<KeyPress-i>', button_10_pressed)
button_10.bind('<ButtonRelease>', button_10_released)
window.bind('<KeyRelease-i>', button_10_released)

button_image_11 = PhotoImage(
    file=relative_to_assets("button_11.png"))
button_11 = Button(
    image=button_image_11,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: set_data_pid(),
    relief="flat"
)
button_11.place(
    x=921.0,
    y=329.0,
    width=50.0,
    height=15.0
)

button_image_12 = PhotoImage(
    file=relative_to_assets("button_12.png"))
button_12 = Button(
    image=button_image_12,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: get_data_pid(),
    relief="flat"
)
button_12.place(
    x=856.0,
    y=329.0,
    width=50.0,
    height=15.0
)

image_image_1 = PhotoImage(
    file=relative_to_assets("image_1.png"))
image_1 = canvas.create_image(
    771.0,
    424.0,
    image=image_image_1
)

image_image_2 = PhotoImage(
    file=relative_to_assets("image_2.png"))
image_2 = canvas.create_image(
    294.0,
    424.0,
    image=image_image_2
)

image_image_3 = PhotoImage(
    file=relative_to_assets("image_3.png"))
image_3 = canvas.create_image(
    433.0,
    208.0,
    image=image_image_3
)

image_image_4 = PhotoImage(
    file=relative_to_assets("image_4.png"))
image_4 = canvas.create_image(
    155.0,
    208.0,
    image=image_image_4
)

image_image_5 = PhotoImage(
    file=relative_to_assets("image_5.png"))
image_5 = canvas.create_image(
    671.0,
    58.0,
    image=image_image_5
)

image_image_6 = PhotoImage(
    file=relative_to_assets("image_6.png"))
image_6 = canvas.create_image(
    232.0,
    54.0,
    image=image_image_6
)

entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    894.6806564331055,
    485.0,
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
    x=835.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    767.6806564331055,
    485.0,
    image=entry_image_2
)
entry_2 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_2.place(
    x=708.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_3 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_3 = canvas.create_image(
    640.6806564331055,
    485.0,
    image=entry_image_3
)
entry_3 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_3.place(
    x=581.916259765625,
    y=473.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_4 = PhotoImage(
    file=relative_to_assets("entry_4.png"))
entry_bg_4 = canvas.create_image(
    894.6806564331055,
    407.0,
    image=entry_image_4
)
entry_4 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_4.place(
    x=835.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_5 = PhotoImage(
    file=relative_to_assets("entry_5.png"))
entry_bg_5 = canvas.create_image(
    767.6806564331055,
    407.0,
    image=entry_image_5
)
entry_5 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_5.place(
    x=708.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_6 = PhotoImage(
    file=relative_to_assets("entry_6.png"))
entry_bg_6 = canvas.create_image(
    640.6806564331055,
    407.0,
    image=entry_image_6
)
entry_6 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_6.place(
    x=581.916259765625,
    y=395.0,
    width=117.52879333496094,
    height=22.0
)

entry_image_7 = PhotoImage(
    file=relative_to_assets("entry_7.png"))
entry_bg_7 = canvas.create_image(
    433.0,
    269.0,
    image=entry_image_7
)
entry_7 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_7.place(
    x=341.0,
    y=257.0,
    width=184.0,
    height=22.0
)

entry_image_8 = PhotoImage(
    file=relative_to_assets("entry_8.png"))
entry_bg_8 = canvas.create_image(
    433.0,
    199.0,
    image=entry_image_8
)
entry_8 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_8.place(
    x=341.0,
    y=187.0,
    width=184.0,
    height=22.0
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
    155.0,
    199.0,
    image=entry_image_10
)
entry_10 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0,
    justify="center",
    font=("Arial ", 15)
)
entry_10.place(
    x=63.0,
    y=187.0,
    width=184.0,
    height=22.0
)

encr_text = canvas.create_text(
    465,
    470.0,
    anchor="nw",
    text="encr",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

roll_text = canvas.create_text(
    350,
    470.0,
    anchor="nw",
    text="roll",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

pitch_text = canvas.create_text(
    220,
    470.0,
    anchor="nw",
    text="pitch",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

yaw_text = canvas.create_text(
    90,
    470.0,
    anchor="nw",
    text="yaw",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

encl_text = canvas.create_text(
    465,
    392.0,
    anchor="nw",
    text="encl",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

usr_text = canvas.create_text(
    338,
    392.0,
    anchor="nw",
    text="usr",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

usf_text = canvas.create_text(
    210,
    392.0,
    anchor="nw",
    text="usf",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

usl_text = canvas.create_text(
    80,
    392.0,
    anchor="nw",
    text="usl",
    fill="#000000",
    font=("ABeeZee Regular", 20 * -1)
)

current_mode = canvas.create_text(
    601.0,
    53.0,
    anchor="nw",
    text="Idle",
    fill="#000000",
    font=("ABeeZee Regular", 15 * -1)
)

window.resizable(False, False)

def run_socket_client():
    try:
        sio.connect(SERVER_URL)
        sio.wait()
    except Exception as e:
        print("Error: ", e)

socket_thread = threading.Thread(target=run_socket_client)
socket_thread.daemon = True
socket_thread.start()

if __name__ == "__main__":
    update_dashboard()
    window.mainloop()
    