import socket
from message import MessageHandler
from threading import Thread


class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.clients = []

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)

    def sendAll(self, msg):
        for client in self.clients:
            client.send(msg)

    def accept(self):
        conn, addr = self.sock.accept()
        client = MessageHandler(conn)
        self.clients.append(client)
        return client

    def remove(self, client):
        client.sock.close()
        self.clients.remove(client)

    def handle(self, client):
        try:
            while True:
                msg = client.recv()
                if msg:

                    if 'graphData' in msg.msg_type:
                        self.sendAll(msg)

                    elif msg.msg_type == 'showGraph':
                        self.sendAll(msg)

                    elif msg.msg_type == 'quit':
                        self.remove(client)
                        break

        except Exception as e:
            print("Error raised:", e)
            self.remove(client)


def main():

    with open('IP/ipAdd', 'r') as f: ip = f.read()
    server = Server(ip, 5000)

    print("SERVER up on IP: ", ip, " Port: 5000")

    while True:
        client = server.accept()
        Thread(target=server.handle, args=(client,)).start()


if __name__ == '__main__':
    main()
