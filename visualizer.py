import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

import sys

class worldMap():
    def __init__(self, x_size, y_size, goalX, goalY, chargers, costmap):
        self.x_size = x_size
        self.y_size = y_size
        self.goalX = goalX
        self.goalY = goalY
        self.chargers = chargers
        self.costmap = costmap


def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size_, y_size_ = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'R', "Expected 'R' in the third line"
        robotX, robotY, robotC = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'B', "Expected 'B' in the fifth line"
        robotCMax = map(int, file.readline().strip())
        
        assert file.readline().strip() == 'G', "Expected 'G' in the third line"
        goalX, goalY = map(int, file.readline().strip().split(','))

        assert file.readline().strip() == 'C', "Expected 'C' in the fifth line"
        chargers = []
        line = file.readline().strip()
        while line != 'M':
            x, y = map(float, line.split(','))
            chargers.append({'x': x, 'y': y})
            line = file.readline().strip()

        costmap_ = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap_.append(row)

        costmap_ = np.asarray(costmap_).T

    return x_size_, y_size_, robotX, robotY, robotC, robotCMax, goalX, goalY, chargers, costmap_

def parse_robot_trajectory_file(filename):
    robot_traj = []
    with open(filename, 'r') as file:
        for line in file:
            t, x, y, c = map(int, line.strip().split(','))
            robot_traj.append({'t': t, 'x': x, 'y': y, 'c': c})

    return robot_traj

SPEEDUP = 10

def colored_line_between_pts(x, y, c, ax, **lc_kwargs):
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, **lc_kwargs)
        lc.set_array(c)
        return ax.add_collection(lc)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)

    x_size_, y_size_, robotX, robotY, robotC, robotCMax, goalX, goalY, chargers, costmap = parse_mapfile(sys.argv[1])

    robot_trajectory = parse_robot_trajectory_file('robot_trajectory.txt')

    fig, ax = plt.subplots()

    ax.imshow(costmap, zorder=0, cmap='jet')

    lc = colored_line_between_pts([p['x'] for p in robot_trajectory], 
                                  [p['y'] for p in robot_trajectory], 
                                  [p['c'] for p in robot_trajectory], 
                                  ax, linewidth=2, cmap="viridis")

    def init():
        lc.set_segments([])
        return lc


    def update(frame, xarr, yarr):
        frame *= SPEEDUP
        x = xarr[:frame + 1]
        y = yarr[:frame + 1]
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc.set_segments(segments)

        return lc

    

    ani = FuncAnimation(fig, update, fargs=([p['x'] for p in robot_trajectory], [p['y'] for p in robot_trajectory]),
                        frames=(len(robot_trajectory) - 1) // SPEEDUP, init_func=init, blit=False,
                        interval=1)

    plt.legend()
    plt.show()
    ani.save("myGIF.gif")
