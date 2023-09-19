import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from math import degrees, pi


def graph(data):

    keys = list(data.keys())

    ax = plt.gca()

    for key in keys:
        if 'x' == key[0]:
            chr = key[1:]
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


def log_graph(log, wheelRadius, DBM):

    halfDBM = DBM / 2
    wheelCircumference = 2 * pi * wheelRadius

    formatted_log = {}
    for key, value in log[0].items():
        if isinstance(value, dict):
            formatted_log[key] = {}
            for subkey, subvalue in value.items():
                if isinstance(subvalue, dict):
                    formatted_log[key][subkey] = {}
                    for subsubkey, subsubvalue in subvalue.items():
                        formatted_log[key][subkey][subsubkey] = [subsubvalue]
                elif isinstance(subvalue, list) or isinstance(subvalue, tuple):
                    formatted_log[key][subkey] = [[i] for i in subvalue]
                else:
                    formatted_log[key][subkey] = [subvalue]
        elif isinstance(value, list) or isinstance(value, tuple):
            formatted_log[key] = [[i] for i in value]
        else:
            formatted_log[key] = [value]
    for timestamp in log[1:]:
        for key, value in timestamp.items():
            if isinstance(value, dict):
                for subkey, subvalue in value.items():
                    if isinstance(subvalue, dict):
                        for subsubkey, subsubvalue in subvalue.items():
                            formatted_log[key][subkey][subsubkey].append(subsubvalue)
                    elif isinstance(subvalue, list) or isinstance(subvalue, tuple):
                        for i, subsubvalue in enumerate(subvalue):
                            formatted_log[key][subkey][i].append(subsubvalue)
                    else:
                        formatted_log[key][subkey].append(subvalue)
            elif isinstance(value, list) or isinstance(value, tuple):
                for i, subvalue in enumerate(value):
                    formatted_log[key][i].append(subvalue)
            else:
                formatted_log[key].append(value)

    # Creating plots
    pose_plot = plt.subplot2grid((3, 3), (0, 0), 2, 2)
    pose_plot.set_title("Pose")
    velocity_plot = plt.subplot2grid((3, 3), (2, 0), 2, 1)
    velocity_plot.set_title("Velocity")
    omega_plot = plt.subplot2grid((3, 3), (2, 1), 2, 1)
    omega_plot.set_title("Omega")
    theata_plot = plt.subplot2grid((4, 3), (0, 2), 1, 1)
    theata_plot.set_title("Theata")
    leftAcc_plot = plt.subplot2grid((8, 3), (2, 2), 1, 1)
    leftAcc_plot.set_title("Acc left", fontsize=8, y=0.9)
    leftAcc_plot.tick_params(bottom=False, labelbottom=False)
    rightAcc_plot = plt.subplot2grid((8, 3), (3, 2), 1, 1)
    rightAcc_plot.set_title("Acc right", fontsize=8, y=0.9)
    Vl_plot = plt.subplot2grid((4, 3), (2, 2), 1, 1)
    Vl_plot.set_title("Vl")
    Vr_plot = plt.subplot2grid((4, 3), (3, 2), 1, 1)
    Vr_plot.set_title("Vr")

    if 'robot' in formatted_log:
        robot = formatted_log['robot']

        if 'Pose' in robot:
            points = np.array([formatted_log['robot']['Pose'][0], formatted_log['robot']['Pose'][1]]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
            lc.set_array(np.linspace(0, len(formatted_log['robot']['Pose'][0]), len(formatted_log['robot']['Pose'][0])))
            pose_plot.add_collection(lc)

            theata_plot.plot(formatted_log['time'], [degrees(i) for i in formatted_log['robot']['Pose'][2]])

            omega_plot.plot(formatted_log['time'], [degrees(i)])

        if 'Phi' in robot:
            deltaTime = [t - robot['time'][i-1] for i, t in enumerate(robot['time'][1:])]

            Vl, Vr = [0], [0]
            for i, Phi in enumerate(robot['Phi'][1:]):
                Vl.append(Phi[0]/360 * wheelCircumference / deltaTime[i])
                Vr.append(Phi[1]/360 * wheelCircumference / deltaTime[i])

            Vl_plot.plot(robot['time'], Vl)
            Vr_plot.plot(robot['time'], Vr)

            velocity_plot.plot(robot['time'], [sum(V)/2 for V in zip(Vl, Vr)])

            AccL, AccR = [0], [0]
            for i, V in enumerate(Vl[1:]):
                AccL.append((V - Vl[i]) / deltaTime[i])
                AccR.append((Vr[i] - Vr[i]) / deltaTime[i])

            leftAcc_plot.plot(robot['time'], AccL)
            rightAcc_plot.plot(robot['time'], AccR)

    if 'waypoint' in formatted_log:
        waypoints = formatted_log['waypoint']

        if 'x' in waypoints:
            points = np.array([formatted_log['waypoint']['x'], formatted_log['waypoint']['y']]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=plt.get_cmap('hsv'))
            lc.set_array(np.linspace(0, len(formatted_log['waypoint']['x']), len(formatted_log['waypoint']['x'])))
            pose_plot.add_collection(lc)

        if 'V' in waypoints:
            velocity_plot.plot(formatted_log['time'], formatted_log['waypoint']['V'])

        if 'omega' in waypoints:
            omega_plot.plot(formatted_log['time'], [degrees(i) for i in formatted_log['waypoint']['omega']])

        if 'theata' in waypoints:
            theata_plot.plot(formatted_log['time'], [degrees(i) for i in formatted_log['waypoint']['theata']])

        if 'leftAcc' in waypoints:
            leftAcc_plot.plot(formatted_log['time'], formatted_log['waypoint']['leftAcc'])

        if 'rightAcc' in waypoints:
            rightAcc_plot.plot(formatted_log['time'], formatted_log['waypoint']['rightAcc'])

        if 'V' in waypoints and 'omega' in waypoints:
            Vl_plot.plot(formatted_log['time'], [V - omega*halfDBM for V,
                                                 omega in zip(formatted_log['waypoint']['V'],
                                                              formatted_log['waypoint']['omega'])])

            Vr_plot.plot(formatted_log['time'], [V + omega*halfDBM for V,
                                                 omega in zip(formatted_log['waypoint']['V'],
                                                              formatted_log['waypoint']['omega'])])

    pose_plot.autoscale_view()
    plt.subplots_adjust(left=0.03, bottom=0.04, right=0.985, top=0.96, wspace=0.125, hspace=0.5)
    plt.show()


def main():
    print("Starting...")

    graph_type = 'log'

    if graph_type == 'log':
        with open('Tools/Comms/Logs/runtime.log', 'r') as file:
            log_graph(eval(file.read()), 8.16, 9.5)
    else:
        with open('Tools/Data/graph.data', 'r') as file:
            graph(eval(file.read()))
    print("Done!")


if __name__ == '__main__':
    main()
