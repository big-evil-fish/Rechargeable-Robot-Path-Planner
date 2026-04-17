/*=================================================================
 *
 * planner.cpp — naive greedy and stupid planner
 *
 *=================================================================*/
#include "planner.h"
#include <cstdlib>
#include <vector>
#include <unordered_set>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * (XSIZE) + (X - 1))
#define OBSTACLE 1

//____ROBOT CONSTANTS_____
constexpr int SENSOR_RAD = 2;
constexpr int MAX_BATTERY = 100;
constexpr int STEP_COST = 1; //right now it's 1 for all neighbors
constexpr int INF = std::numeric_limits<int>::max() / 4;

///
static int cell_free(int* map, int x_size, int y_size, int x, int y)
{
    if (x < 1 || x > x_size || y < 1 || y > y_size)
        return 0;
    return map[GETMAPINDEX(x, y, x_size, y_size)] == 1;
}
//_____ STRUCTS_____

/// basic cell type instead of pairs
struct Cell {
    int x, y;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Cell& o) const { return !(*this == o); }
};
struct CellHash {
    size_t operator()(const Cell& c) const noexcept {
        return (static_cast<size_t>(c.x) << 20) ^ static_cast<size_t>(c.y);
    }
};

struct PQNode {
    int f;
    int tiebreak;
    Cell c;
    bool operator>(const PQNode& o) const {
        if (f != o.f) return f > o.f;
        return tiebreak > o.tiebreak;
    }
};

/* _______STATE! ______*/

//static, just storing here for convenience
int x_size = 0, y_size = 0;
std::vector<int>  static_map; //TODO: remember to set this

//basic state stuff
std::vector<char> sensed; //specifically which cells have we SEEN ourselves
std::unordered_set<Cell, CellHash> known_outlets; //

std::unordered_set<Cell, CellHash> last_outlet_set; //to check if we've updated

std::vector<int>  d_out; //dist to nearest outlet
std::vector<int>  nearest_outlet_idx;
std::vector<Cell> outlet_list; //ordered version of known_outlets for better checking
std::vector<std::vector<int>> D_outlet; //all-pairs of best distances between outlets!

//plan specific state info
std::vector<Cell> current_plan;
Cell plan_target = {-1, -1};
bool plan_valid  = false;
 
bool initialized = false;

/* SMALL HELPERS! */
inline int idx(int x, int y)          { return y * x_size + x; }
inline bool in_bounds(int x, int y)   { return x >= 0 && x < x_size && y >= 0 && y < y_size; }
inline bool in_bounds(const Cell& c)  { return in_bounds(c.x, c.y); }

inline bool traversable(int x, int y) {
    if (!in_bounds(x, y)) return false;
    return static_map[idx(x, y)] != OBSTACLE;
}
inline bool traversable(const Cell& c) { return traversable(c.x, c.y); }

constexpr int NDX[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
constexpr int NDY[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

//_________________________________


/*literally just dijkstra that runs out to radius max_battery so 
you can get all outlets that can reach each other*/
std::vector<int> bounded_dijkstra(const Cell& source, int max_dist) {
    std::vector<int> dist(x_size * y_size, INF);
    if (!traversable(source)) return dist;
    dist[idx(source.x, source.y)] = 0;
 
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
    pq.push({0, 0, source});
 
    while (!pq.empty()) {
        PQNode top = pq.top(); pq.pop();
        if (top.f > dist[idx(top.c.x, top.c.y)]) continue;
        if (top.f >= max_dist) continue;
        for (int k = 0; k < 8; ++k) {
            int nx = top.c.x + NDX[k], ny = top.c.y + NDY[k];
            if (!traversable(nx, ny)) continue;
            int nd = top.f + STEP_COST;
            if (nd <= max_dist && nd < dist[idx(nx, ny)]) {
                dist[idx(nx, ny)] = nd;
                pq.push({nd, 0, {nx, ny}});
            }
        }
    }
    return dist;
}


//figures out all-pairs between outlets specifically
/*
COMPUTES outlet-to-outlet all-pairs shortest paths
*/
void recompute_outlet_structures(){
    const int N = x_size * y_size;
    std::fill(d_out.begin(), d_out.end(), INF);
    std::fill(nearest_outlet_idx.begin(), nearest_outlet_idx.end(), -1);

    outlet_list.assign(known_outlets.begin(), known_outlets.end());
    const int O = static_cast<int>(outlet_list.size());

    if (O == 0) { //occurs once at start
        D_outlet.clear();
        return;
    }
    //outlet-to-outlet dists
    //multi-source dijkstra
    //sort of the exact same as BFS here (cost is 1) - can maybe replace this later?
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
    for (int i = 0; i < O; ++i) {
        const Cell& o = outlet_list[i];
        if (!in_bounds(o)) continue;
        d_out[idx(o.x, o.y)] = 0;
        nearest_outlet_idx[idx(o.x, o.y)] = i;
        pq.push({0, 0, o});
    }
    while (!pq.empty()) {
        PQNode top = pq.top(); pq.pop();
        int ci = idx(top.c.x, top.c.y);
        if (top.f > d_out[ci]) continue;
        if (top.f >= MAX_BATTERY) continue;
        for (int k = 0; k < 8; ++k) {
            int nx = top.c.x + NDX[k], ny = top.c.y + NDY[k];
            if (!traversable(nx, ny)) continue;
            int ni = idx(nx, ny);
            int nd = top.f + STEP_COST;
            if (nd <= MAX_BATTERY && nd < d_out[ni]) {
                d_out[ni] = nd;
                nearest_outlet_idx[ni] = nearest_outlet_idx[ci];
                pq.push({nd, 0, {nx, ny}});
            }
        }
    }
    //outlet to outlet graph
    std::vector<std::vector<int>> edges(O, std::vector<int>(O, INF));
    for (int i = 0; i < O; ++i) edges[i][i] = 0;
    for (int i = 0; i < O; ++i) {
        auto dists = bounded_dijkstra(outlet_list[i], MAX_BATTERY);
        for (int j = 0; j < O; ++j) {
            if (i == j) continue;
            const Cell& oj = outlet_list[j];
            int d = dists[idx(oj.x, oj.y)];
            if (d <= MAX_BATTERY) edges[i][j] = d;
        }
    }

    //floyd-warshall for final all_pairs
    D_outlet = edges;
    for (int k = 0; k < O; ++k) {
        for (int i = 0; i < O; ++i) {
            if (D_outlet[i][k] >= INF) continue;
            for (int j = 0; j < O; ++j) {
                if (D_outlet[k][j] >= INF) continue;
                int via = D_outlet[i][k] + D_outlet[k][j];
                if (via < D_outlet[i][j]) D_outlet[i][j] = via;
            }
        }
    }
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

    //_______SECTION 1: just updating info based on new info ________________

    //updating sensed
    for (int dx = -SENSOR_RAD; dx <= SENSOR_RAD; dx++){
        for (int dy = -SENSOR_RAD; dy <= SENSOR_RAD; dy++){
            int x = robotposeX+dx, y = robotposeY+dy;
            if (in_bounds(x, y)) sensed[idx(x, y)] = 1;
        }
    }
    //updating outlets
    for (int i = 0; i < num_visible_chargers; i++){
        Cell o{visible_charger_x[i], visible_charger_y[i]};
        if (in_bounds(o)) known_outlets.insert(o);
    }


    //_________SECTION 2: updating outlet path knowledge _____________
    //we're keeping track of shortest paths between outlets but these may update
    if (known_outlets.size() != last_outlet_set.size()) {
        recompute_outlet_structures();
        last_outlet_set = known_outlets;
        //need to replan if new outlet discovered
        current_plan.clear();
        plan_target = {-1, -1};
        plan_valid  = false;
    }

    /* 
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
