class Message:
    def __init__(self, msg_type, msg_data):
        self.msg_type = msg_type
        self.msg_data = msg_data

    def __repr__(self):
        return "Type: " + self.msg_type + ", Data: " + self.msg_data

    def __str__(self):
        return self.__repr__()

    def send(self, sock):
        data_encoded = self.msg_type.encode()
        data_encoded_len = str(len(data_encoded))
        sock.send((' ' * (32-len(data_encoded_len)) + data_encoded_len).encode())
        sock.sendall(data_encoded)

        data_encoded = self.msg_data.encode()
        data_encoded_len = str(len(data_encoded))
        sock.send((' ' * (32-len(data_encoded_len)) + data_encoded_len).encode())
        sock.sendall(data_encoded)

    @classmethod
    def recv(cls, sock):
        temp = sock.recv(32).decode().replace(' ', '')
        if temp != '':
            msg_type = sock.recv(int(temp)).decode()

            temp = sock.recv(32).decode().replace(' ', '')
            if temp != '':
                msg_data = sock.recv(int(temp)).decode()
                return cls(msg_type, msg_data)


class MessageHandler:
    def __init__(self, sock):
        self.sock = sock

    def send(self, msg):
        msg.send(self.sock)

    def recv(self):
        return Message.recv(self.sock)
