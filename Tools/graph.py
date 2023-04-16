import matplotlib.pyplot as plt


def graph(data: dict):

    keys = list(data.keys())

    for key in keys:
        if 'x' in key:
            num = key[1]
            plt.plot(data[key], data['y'+num], label=num)

    if 'title' in data:
        plt.title(data['title'])
    if 'lables' in data:
        plt.xlabel(data['lables'][0])
        plt.ylabel(data['lables'][1])

    plt.legend()
    plt.show()


def main():
    print("Starting...")
    with open('Data/graph.data', 'r') as file:
        graph(eval(file.read()))
    print("Done!")


if __name__ == '__main__':
    main()
