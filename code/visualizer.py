import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap

import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size_, y_size_ = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'D', "Expected D in the third line"
        sensorRange = int(file.readline().strip())

        assert file.readline().strip() == 'R', "Expected 'R' in the fifth line"
        robotX, robotY = map(int, file.readline().strip().split(','))
        robotX -=1
        robotY -=1

        assert file.readline().strip() == 'B', "Expected 'B' in the seventh line"
        robotCMax, robotC = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'K', "Expected 'K' in the ninth line"
        chargers = []
        numChargers = file.readline().strip()
        line = file.readline().strip()
        while line != 'G':
            x, y = map(int, line.split(','))
            x -=1
            y -=1
            chargers.append({'x': x, 'y': y})
            line = file.readline().strip()
        assert line == 'G', "Expected 'G' in the eleventh line"
        goalX, goalY = map(int, file.readline().strip().split(','))
        goalX -=1
        goalY -=1
        assert file.readline().strip() == 'M', "Expected 'M'"
        costmap_ = []
        for line in file:
            row = list(map(int, line.strip().split(',')))
            costmap_.append(row)

        costmap_ = np.asarray(costmap_).T

    return x_size_, y_size_, robotX, robotY, robotC, robotCMax, goalX, goalY, sensorRange, chargers, costmap_
    #return worldMap(x_size_, y_size_, robotX, robotY, robotC, robotCMax, goalX, goalY, sensorRange, chargers, costmap_)

def parse_robot_trajectory_file(filename):
    robot_traj = []
    with open(filename, 'r') as file:
        for line in file:
            t, x, y, c = map(int, line.strip().split(','))
            x -=1
            y -=1
            robot_traj.append({'t': t, 'x': x, 'y': y, 'c': c})

    return robot_traj

SPEEDUP = 1

def colored_line_between_pts(x, y, c, ax, **lc_kwargs):
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, **lc_kwargs)
        lc.set_array(c)
        return ax.add_collection(lc)

def buildChargermap(chargers, costmap):
    chargermap = np.zeros_like(costmap)
    for charger in chargers:
        #print(f"{charger['x']}, {charger['y']}")
        chargermap[charger['x'], charger['y']] = 1
    return chargermap

def buildKnownMap(x, y, x_size, y_size, sensorRange):
    knownMap = np.zeros((x_size, y_size))
    for pos in range(1, len(x)):
        for i in range(-1*sensorRange, 1+ sensorRange):
            for j in range(-1*sensorRange, 1+ sensorRange):
                if (x[pos]+i)>=0 and (x[pos]+i)<x_size \
                    and (y[pos]+j)>=0 and (y[pos]+j)<y_size:
                    knownMap[x[pos]+i, y[pos]+j]=1
    return knownMap


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)

    x_size, y_size, robotX, robotY, robotC, robotCMax, goalX, goalY, sensorRange, chargers, costmap = parse_mapfile(sys.argv[1])
    knownMap = np.zeros_like(costmap)
    chargermap = buildChargermap(chargers, costmap)
    chargermap[goalX, goalY] = 2
    #print(chargermap)

    robot_trajectory = parse_robot_trajectory_file('robot_trajectory.txt')

    fig, ax = plt.subplots()

    ax.matshow(costmap, zorder=0, cmap='binary')

    #ax.imshow(knownMap, zorder=2, cmap=ListedColormap([(0,0,0,0.3), (1, 1, 1, 0.1)]))

    cxs = [c['x'] for c in chargers]
    cys = [c['y'] for c in chargers]
    if cxs:
        ax.scatter(cxs, cys, c="blue", s=200, marker="*", zorder=3, label="chargers", edgecolors="k", linewidths=0.5)

    ax.scatter([goalX], [goalY], c="gold", s=200, marker="s", zorder=4, label="goal", edgecolors="k", linewidths=0.5)
    
    lc = colored_line_between_pts([p['x'] for p in robot_trajectory], 
                                  [p['y'] for p in robot_trajectory], 
                                  [p['c'] for p in robot_trajectory], 
                                  ax, linewidth=4, cmap="RdYlGn", zorder=3)
    sc = ax.scatter([p['x'] for p in robot_trajectory], 
                    [p['y'] for p in robot_trajectory], 
                    c=[p['c'] for p in robot_trajectory], 
                    cmap="RdYlGn", s=24, edgecolors="k",
                    linewidths=0.4, zorder=4)

    def init():
        lc.set_segments([])
        knownMap = np.zeros_like(costmap)
        return lc, knownMap

    def update(frame, xarr, yarr, x_size, y_size, sensorRange):
        frame = frame+1 if SPEEDUP == 1 else frame*SPEEDUP
        
        # multicolor line following robot path
        x = xarr[:frame + 1]
        y = yarr[:frame + 1]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        data = np.stack([x, y]).T
        sc.set_offsets(data)
        lc.set_segments(segments)

        # known map visulization
        #knownMap = buildKnownMap(x, y, x_size, y_size, sensorRange)
        #print(sum(sum(knownMap)))

        return (lc, sc)

    ani = FuncAnimation(fig, update, fargs=([p['x'] for p in robot_trajectory],
                        [p['y'] for p in robot_trajectory], x_size, y_size, sensorRange),
                        frames=(len(robot_trajectory) - 1)//SPEEDUP, init_func=init, blit=False,
                        interval=1)
    plt.colorbar(lc)
    plt.show()
    ani.save("myGIF.gif")