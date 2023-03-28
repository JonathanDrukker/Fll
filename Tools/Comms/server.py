import socket
from message import MessageHandler
from threading import Thread


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.handlers = []

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)

    def sendEveryone(self, msg):
        for handler in self.handlers:
            handler.send(msg)

    def accept(self):
        conn, addr = self.sock.accept()
        handeler = MessageHandler(conn)
        self.handlers.append(handeler)
        return handeler

    def handle(self, handler):
        try:
            x, y = [], []
            while True:
                msg = handler.recv()
                if msg:
                    if msg.msg_type == 'quit':
                        break
                    elif msg.msg_type == 'pos':
                        lst = eval(msg.msg_data)
                        x.append(lst[0]); y.append(lst[1])
                    elif msg.msg_type == 'save':
                        with open('G:\My Drive\GitHub\Fll\Tools\graph.data', 'w') as f:
                            f.write(str({'x0': x, 'y0': y}))
                            print("Saved")

        finally:
            self.handlers.remove(handler)
            handler.sock.close()


def main():

    with open('ipAdd', 'r') as f: ip = f.read()
    server = Server(ip, 5000)

    print("SERVER up on IP: ", ip, " Port: 5000")

    while True:
        handler = server.accept()
        Thread(target=server.handle, args=(handler,)).start()


if __name__ == '__main__':
    main()
