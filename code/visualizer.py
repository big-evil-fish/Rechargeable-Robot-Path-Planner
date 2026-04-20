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
    ax.matshow(chargermap, zorder=1,cmap=ListedColormap([(0,0,0,0), (1,0,0,1), (1,0.8,0.6,1)]))

    ax.imshow(knownMap, zorder=2, cmap=ListedColormap([(0,0,0,0.3), (0, 0, 0, 0)]))

    lc = colored_line_between_pts([p['x'] for p in robot_trajectory], 
                                  [p['y'] for p in robot_trajectory], 
                                  [p['c'] for p in robot_trajectory], 
                                  ax, linewidth=4, cmap="winter", zorder=2)

    def init():
        lc.set_segments([])
        return lc

    def update(frame, xarr, yarr, knownMap, x_size, y_size, sensorRange):
        frame = frame+1 if SPEEDUP == 1 else frame*SPEEDUP
        
        # multicolor line following robot path
        x = xarr[:frame + 1]
        y = yarr[:frame + 1]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc.set_segments(segments)

        #x=x[-1]
        #y=y[-1]
        #print(f"{x}, {y}")
        # known map visulization
        for pos in range(1, frame+1):
            for i in range(-1*sensorRange, 1+ sensorRange):
                for j in range(-1*sensorRange, 1+ sensorRange):
                    if (x[pos]+i)>=0 and (x[pos]+i)<x_size-1 \
                        and (y[pos]+j)>=0 and (y[pos]+j)<y_size-1:
                        knownMap[x[pos]+i, y[pos]+j]=1

        return lc, knownMap

    ani = FuncAnimation(fig, update, fargs=([p['x'] for p in robot_trajectory],
                        [p['y'] for p in robot_trajectory], knownMap, x_size, y_size, sensorRange),
                        frames=(len(robot_trajectory) - 1)//SPEEDUP, init_func=init, blit=False,
                        interval=1)

    plt.show()
    ani.save("myGIF.gif")