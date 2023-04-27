from Comms.client import Client
from graph import graph

with open('Comms/IP/ipAdd', 'r') as f:
    ip = f.read()
print("Connecting to server... IP: ", ip)
client = Client(ip, 5000)
print("Connected to server! IP: ", ip)

X0, Y0 = [], []
X1, Y1 = [], []

while True:
    msg = client.recv()

    if 'graphData' in msg.msg_type:
        num = msg.msg_type[-1]
        x, y = eval(msg.msg_data)
        if num == '0':
            X0.append(x)
            Y0.append(y)
        elif num == '1':
            X1.append(x)
            Y1.append(y)

    elif msg.msg_type == 'showGraph':
        graph({'xP': X0, 'yP': Y0, 'xW': X1, 'yW': Y1, 'cP': 'hsv', 'cW': 'hsv'})
        X0, Y0 = [], []
        X1, Y1 = [], []

client.close()
