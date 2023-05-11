from Comms.client import Client
from graph import log_graph

with open('Comms/IP/ipAdd', 'r') as f:
    ip = f.read()
print("Connecting to server... IP: ", ip)
client = Client(ip, 5000)
print("Connected to server! IP: ", ip)

log = []

while True:
    msg = client.recv()
    if msg.msg_type == 'log':
        log.append(eval(msg.msg_data))
    elif msg.msg_type == 'show':
        log_graph(log, *eval(msg.msg_data))
        log = []
    elif msg.msg_type == 'quit':
        break
