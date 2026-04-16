/*=================================================================
 *
 * runtest.cpp
 *
 *=================================================================*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>

#include "planner.h"

/*
Map format:

N
x_size,y_size
D
detection_radius
R
robotposeX,robotposeY
B
charge_max,charge
K
n_chargers
charger_x[1],charger_y[1]
...
charger_x[n_chargers],charger_y[n_chargers]
G
goalposeX,goalposeY
M
grid
*/

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "runtest takes exactly one command line argument: the map file" << std::endl;
        return -1;
    }

    std::cout << "Reading problem definition from: " << argv[1] << std::endl;

    std::ifstream myfile;
    myfile.open(argv[1]);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
        return -1;
    }

    char letter;
    std::string line;
    int x_size, y_size;

    myfile >> letter;
    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }

    myfile >> x_size >> letter >> y_size;
    std::cout << "map size: " << x_size << letter << y_size << std::endl;

    myfile >> letter;
    if (letter != 'D')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    int detection_radius;
    myfile >> detection_radius;

    int robotposeX, robotposeY;
    myfile >> letter;
    if (letter != 'R')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }

    myfile >> robotposeX >> letter >> robotposeY;
    std::cout << "robot pose: " << robotposeX << letter << robotposeY << std::endl;

    myfile >> letter;
    if (letter != 'B')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    int charge_max;
    int charge;
    myfile >> charge_max >> letter >> charge;

    myfile >> letter;
    if (letter != 'K')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    int n_chargers;
    myfile >> n_chargers;
    std::vector<int> charger_x;
    std::vector<int> charger_y;
    for (int i = 0; i < n_chargers; ++i)
    {
        int cx, cy;
        myfile >> cx >> letter >> cy;
        charger_x.push_back(cx);
        charger_y.push_back(cy);
    }

    myfile >> letter;
    if (letter != 'G')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    int goalposeX, goalposeY;
    myfile >> goalposeX >> letter >> goalposeY;

    myfile >> letter;
    if (letter != 'M')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }

    int max_time = x_size * y_size;

    int* map = new int[x_size*y_size];
    for (size_t i=0; i<x_size; i++)
    {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t j=0; j<y_size; j++)
        {
            double value;
            ss >> value;
            int raw = (int)value;
            map[j*x_size+i] = (raw == 0) ? 1 : 0;
            if (j != y_size-1) ss.ignore();
        }
    }

    myfile.close();
    std::cout << "\nRunning planner" << std::endl;

    int curr_time = 0;
    int* action_ptr = new int[2];
    int newrobotposeX, newrobotposeY;

    int numofmoves = 0;
    bool goal_reached = false;
    int pathcost = 0;

    int num_charger = (int)charger_x.size();
    std::vector<int> detected_x;
    std::vector<int> detected_y;
    detected_x.reserve((size_t)num_charger);
    detected_y.reserve((size_t)num_charger);

    std::ofstream output_file("robot_trajectory.txt");
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    output_file << curr_time << "," << robotposeX << "," << robotposeY << "," << charge << std::endl;

    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();

        detected_x.clear();
        detected_y.clear();
        for (int i = 0; i < num_charger; ++i)
        {
            int dx = std::abs(charger_x[i] - robotposeX);
            int dy = std::abs(charger_y[i] - robotposeY);
            if (std::max(dx, dy) <= detection_radius)
            {
                detected_x.push_back(charger_x[i]);
                detected_y.push_back(charger_y[i]);
            }
        }
        int n_detected = (int)detected_x.size();
        const int* pdx = n_detected ? detected_x.data() : nullptr;
        const int* pdy = n_detected ? detected_y.data() : nullptr;

        planner(map, x_size, y_size, robotposeX, robotposeY, charge, goalposeX, goalposeY,
                n_detected, pdx, pdy, action_ptr);
        newrobotposeX = action_ptr[0];
        newrobotposeY = action_ptr[1];

        if (newrobotposeX < 1 || newrobotposeX > x_size || newrobotposeY < 1 || newrobotposeY > y_size)
        {
            std::cout << "ERROR: out-of-map robot position commanded\n" << std::endl;
            return -1;
        }

        if (map[(newrobotposeY-1)*x_size + newrobotposeX-1] != 1)
        {
            std::cout << "ERROR: planned action leads to collision\n" << std::endl;
            return -1;
        }

        if (abs(robotposeX-newrobotposeX)>1 || abs(robotposeY-newrobotposeY)>1)
        {
            std::cout << "ERROR: invalid action commanded. robot must move on 8-connected grid.\n" << std::endl;
            return -1;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        int movetime = std::max(1, (int)std::ceil(duration));

        if (newrobotposeX == robotposeX && newrobotposeY == robotposeY)
            numofmoves -= 1;

        if (curr_time + movetime >= max_time)
            break;

        curr_time = curr_time + movetime;
        numofmoves = numofmoves + 1;
        pathcost = pathcost + 1;

        robotposeX = newrobotposeX;
        robotposeY = newrobotposeY;

        charge -= 1;
        for (int i = 0; i < num_charger; ++i)
        {
            if (robotposeX == charger_x[i] && robotposeY == charger_y[i])
            {
                charge = charge_max;
                break;
            }
        }

        output_file << curr_time << "," << robotposeX << "," << robotposeY << "," << charge << std::endl;

        if (charge <= 0)
        {
            std::cout << "ERROR: out of charge\n" << std::endl;
            return -1;
        }

        float thresh = 0.5;
        std::cout << "goalposeX: " << goalposeX << std::endl;
        std::cout << "goalposeY: " << goalposeY << std::endl;
        std::cout << "robotposeX: " << robotposeX << std::endl;
        std::cout << "robotposeY: " << robotposeY << std::endl;
        std::cout << "Time: " << curr_time << std::endl;
        if (abs(robotposeX - goalposeX) <= thresh && abs(robotposeY - goalposeY) <= thresh)
        {
            goal_reached = true;
            break;
        }
    }

    output_file.close();

    std::cout << "\nRESULT" << std::endl;
    std::cout << "goal reached = " << goal_reached << std::endl;
    std::cout << "time taken (s) = " << curr_time << std::endl;
    std::cout << "moves made = " << numofmoves << std::endl;
    std::cout << "path cost = " << pathcost << std::endl;

    delete[] map;

    return 0;
}
