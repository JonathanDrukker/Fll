import matplotlib.pyplot as plt


def main():
    print("Starting...")
    with open('graph.data', 'r') as file:

        data = eval(file.read())
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
    print("Done!")


if __name__ == '__main__':
    main()
