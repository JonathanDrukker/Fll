import matplotlib.pyplot as plt

print("Starting...")
with open('Tools/graph.data', 'r') as file:

    data = eval(file.read())
    keys = list(data.keys())

    for key in keys:
        if 'x' in key:
            num = key[1]
            plt.plot(data[key], data['y'+num], label=num)

    plt.title(data['title'])
    plt.xlabel(data['lables'][0])
    plt.ylabel(data['lables'][1])

plt.legend()
plt.show()
print("Done!")
