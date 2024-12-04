import socketio

sio = socketio.Client()
@sio.event
def connect():
    print('connection established')

@sio.event
def disconnect():
    print('disconnected from server')

@sio.event
def msg(data):
    print('message received : ', data)

sio.connect('http://192.168.43.6:5000')

sio.emit('msg', 'hello')

sio.wait()
