from flask import Flask, request
from flask_socketio import SocketIO
import serial
from serial.tools import list_ports
import time
import threading    

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

serialDevice = None
serialBaudrate = 115200
clientIP = None
clientSID = None

def scanPorts():
    global serialDevice
    while serialDevice is None:
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            try:
                serialDevice = serial.Serial(port.device, serialBaudrate)
                print("Connected to ", port.device)
                break
            except:
                pass
        print("No device found. Retrying...")
        # time.sleep(1)

def handleSerialData():
    global serialDevice, clientIP

    while True:
        if serialDevice is not None:
            try:
                if serialDevice.in_waiting > 0:
                    data = serialDevice.readline().decode('utf-8', errors='ignore').strip()
                    socketio.emit('msg', data, to=clientSID)
            except serial.SerialException:
                serialDevice = None
                data = None
                scanPorts()
            except Exception as e:
                print(f"Error: {e}")
                exit(1)
            # time.sleep(0.1)

@socketio.on('connect')
def handle_connect():
    global clientSID
    global clientIP
    clientSID = request.sid
    clientIP = request.remote_addr
    print(f"Client connected: {clientIP}")

@socketio.on('disconnect')
def handle_disconnect():
    global clientSID
    global clientIP
    print(f"Client disconnected: {clientIP}")
    clientSID = None
    clientIP = None

@socketio.on('msg')
def handle_message(data):
    global serialDevice
    if serialDevice is not None:
        try:
            serialDevice.write(data.encode())
            print(f"data from client: {data}")
        except serial.SerialException:
            serialDevice = None
            scanPorts()
        except Exception as e:
            print(f"Error: {e}")
            # exit(1)

serial_thread = threading.Thread(target=handleSerialData)
serial_thread.daemon = True
serial_thread.start()
scanPorts()

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)