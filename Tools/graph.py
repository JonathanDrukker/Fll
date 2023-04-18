import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def graph(data: dict):

    keys = list(data.keys())

    ax = plt.gca()

    for key in keys:
        if 'x' in key:
            chr = key[1]
            x, y = data[key], data['y'+chr]

            if 'c'+chr in data:
                points = np.array([x, y]).T.reshape(-1, 1, 2)
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                lc = LineCollection(segments, cmap=plt.get_cmap(data['c'+chr]))
                lc.set_array(np.linspace(0, len(x), len(x)))

                ax.add_collection(lc)
            else:
                plt.plot(x, y, label=chr)

    if 'title' in data:
        plt.title(data['title'])
    if 'lables' in data:
        plt.xlabel(data['lables'][0])
        plt.ylabel(data['lables'][1])

    ax.autoscale_view()
    plt.show()


def main():
    print("Starting...")
    with open('Tools/Data/graph.data', 'r') as file:
        graph(eval(file.read()))
    print("Done!")


if __name__ == '__main__':
    main()
