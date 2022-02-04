import socket
import json
from threading import Thread


class RobotClient:
    host = '127.0.0.1'
    port = 65432
    sock = socket.socket()
    infile = None
    reaction = None
    listen_thread = None

    def __init__(self):
        self.sock = socket.socket()
        self.host = '127.0.0.1'
        self.port = 9002
        self.reaction = None
        self.infile = None
        self.listen_thread = None

    def threaded_function(self):
        while True:
            data = self.infile.readline()
            if not data:
                self.disconnect()
                break
            input_t = json.loads(data)
            self.reaction(input_t)

    def connect(self, host="localhost", port=65432):
        self.sock.connect((host, port))

        self.infile = self.sock.makefile()

        self.listen_thread = Thread(target=self.threaded_function)
        self.listen_thread.start()

    def set_reaction(self, function):
        self.reaction = function

    def set_state(self, text):
        m = {"type": "setState", "text": text}
        data = json.dumps(m)
        self.sock.sendall((str(data) + "\n").encode())

    def send(self, list_to_json):
        data = json.dumps(list_to_json)
        self.sock.sendall((str(data) + "\n").encode())

    def disconnect(self):
        self.sock.close()
