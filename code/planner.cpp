/*=================================================================
 *
 * planner.cpp — naive greedy and stupid planner
 *
 *=================================================================*/
#include "planner.h"
#include <cstdlib>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * (XSIZE) + (X - 1))

static int cell_free(int* map, int x_size, int y_size, int x, int y)
{
    if (x < 1 || x > x_size || y < 1 || y > y_size)
        return 0;
    return map[GETMAPINDEX(x, y, x_size, y_size)] == 1;
}

void planner(
    int* map,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int current_charge,
    int goalposeX,
    int goalposeY,
    int num_visible_chargers,
    const int* visible_charger_x,
    const int* visible_charger_y,
    int* action_ptr)
{
    //compiler warning nonsense
    (void)current_charge;
    (void)num_visible_chargers;
    (void)visible_charger_x;
    (void)visible_charger_y;
    //////

    //sec 1: just updating info based on sensing

    /* 
    sec 2: updating outlet knowledge
    we're keeping track of shortest paths between outlets but these may update
    if we've discovered new ones.

    if on outlet: update battery ?
    */

    //sec 3: goal is reachable path

    //sec 4: explore by frontier-based search


    //greedy backup
    int nx = robotposeX;
    int ny = robotposeY;

    if (goalposeX > robotposeX && cell_free(map, x_size, y_size, robotposeX + 1, robotposeY))
        nx = robotposeX + 1;
    else if (goalposeX < robotposeX && cell_free(map, x_size, y_size, robotposeX - 1, robotposeY))
        nx = robotposeX - 1;
    else if (goalposeY > robotposeY && cell_free(map, x_size, y_size, robotposeX, robotposeY + 1))
        ny = robotposeY + 1;
    else if (goalposeY < robotposeY && cell_free(map, x_size, y_size, robotposeX, robotposeY - 1))
        ny = robotposeY - 1;

    action_ptr[0] = nx;
    action_ptr[1] = ny;
}
