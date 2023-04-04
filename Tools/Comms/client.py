import socket
from Comms.message import Message


class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def send(self, msg):
        msg.send(self.sock)

    def recv(self):
        return Message.recv(self.sock)

    def quit(self):
        self.send(Message('quit', ''))
        self.sock.close()


def main():
    with open('ipAdd', 'r') as f:
        ip = f.read()
    client = Client(ip, 5000)
    try:
        while True:
            msg = Message(input('Type: '), input('Data: '))
            client.send(msg)
    finally:
        client.close()


if __name__ == '__main__':
    main()
