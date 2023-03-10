import matplotlib.pyplot as plt
from json import load

try:
    with open('Tools/Graph/graph.json', 'r') as file:

        data = load(file)
        keys = list(data.keys())

        for key in keys:
            if 'x' in key:
                num = key[1]
                plt.plot(data[key], data['y'+num])

        plt.plot(data['x'], data['y'])
        plt.title(data['title'])
        plt.xlabel(data['xlable'])
        plt.ylabel(data['ylable'])

    plt.show()

except Exception as e:
    print(e)
