import matplotlib.pyplot as plt
from json import load


with open('Tools/Graph/graph.json', 'r') as file:
    data = load(file)
    plt.plot(data['x'], data['y'])

plt.xlabel('Seconds')
plt.ylabel('Degrees')
plt.title('Time-Angle')
plt.show()
