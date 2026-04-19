/*=================================================================
 *
 * planner.cpp — naive greedy and stupid planner
 *
 *=================================================================*/
#include "planner.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * (XSIZE) + (X - 1))
// Same encoding as runtest's map[] after load: 1 = traversable, 0 = blocked
#define RUNTEST_MAP_FREE 1

//____ROBOT CONSTANTS_____
constexpr int SENSOR_RAD = 2;
constexpr int MAX_BATTERY = 100;
constexpr int STEP_COST = 1; //right now it's 1 for all neighbors
// ^ if keeping this - switch to BFS
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

struct PlannerState{
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
    const int* map_copy_src = nullptr;
};

PlannerState S;

/* SMALL HELPERS! */
// runtest uses 1-index coordinates for some reason
inline int idx(int x, int y) { return (y - 1) * S.x_size + (x - 1); }
inline bool in_bounds(int x, int y) {
    return x >= 1 && x <= S.x_size && y >= 1 && y <= S.y_size;
}
inline bool in_bounds(const Cell& c)  { return in_bounds(c.x, c.y); }

inline bool traversable(int x, int y) {
    if (!in_bounds(x, y)) return false;
    return S.static_map[idx(x, y)] == RUNTEST_MAP_FREE;
}
inline bool traversable(const Cell& c) { return traversable(c.x, c.y); }

constexpr int NDX[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
constexpr int NDY[8] = {-1,  0,  1, -1, 1, -1, 0, 1};

inline int octile(const Cell& a, const Cell& b) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return std::max(dx, dy) * STEP_COST;
}

//_________________________________


/*literally just dijkstra that runs out to radius max_battery so 
you can get all outlets that can reach each other*/
std::vector<int> bounded_dijkstra(const Cell& source, int max_dist) {
    std::vector<int> dist(S.x_size * S.y_size, INF);
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
    const int N = S.x_size * S.y_size;
    std::fill(S.d_out.begin(), S.d_out.end(), INF);
    std::fill(S.nearest_outlet_idx.begin(), S.nearest_outlet_idx.end(), -1);

    S.outlet_list.assign(S.known_outlets.begin(), S.known_outlets.end());
    const int O = static_cast<int>(S.outlet_list.size());

    if (O == 0) { //occurs once at start
        S.D_outlet.clear();
        return;
    }
    //outlet-to-outlet dists
    //multi-source dijkstra
    //sort of the exact same as BFS here (cost is 1) - can maybe replace this later?
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
    for (int i = 0; i < O; i++) {
        const Cell& o = S.outlet_list[i];
        if (!in_bounds(o)) continue;
        S.d_out[idx(o.x, o.y)] = 0;
        S.nearest_outlet_idx[idx(o.x, o.y)] = i;
        pq.push({0, 0, o});
    }
    while (!pq.empty()) {
        PQNode top = pq.top(); pq.pop();
        int ci = idx(top.c.x, top.c.y);
        if (top.f > S.d_out[ci]) continue;
        if (top.f >= MAX_BATTERY) continue;
        for (int k = 0; k < 8; k++) {
            int nx = top.c.x + NDX[k], ny = top.c.y + NDY[k];
            if (!traversable(nx, ny)) continue;
            int ni = idx(nx, ny);
            int nd = top.f + STEP_COST;
            if (nd <= MAX_BATTERY && nd < S.d_out[ni]) {
                S.d_out[ni] = nd;
                S.nearest_outlet_idx[ni] = S.nearest_outlet_idx[ci];
                pq.push({nd, 0, {nx, ny}});
            }
        }
    }
    //outlet to outlet graph
    std::vector<std::vector<int>> edges(O, std::vector<int>(O, INF));
    for (int i = 0; i < O; i++) edges[i][i] = 0;
    for (int i = 0; i < O; i++) {
        auto dists = bounded_dijkstra(S.outlet_list[i], MAX_BATTERY);
        for (int j = 0; j < O; j++) {
            if (i == j) continue;
            const Cell& oj = S.outlet_list[j];
            int d = dists[idx(oj.x, oj.y)];
            if (d <= MAX_BATTERY) edges[i][j] = d;
        }
    }

    //floyd-warshall on graph for final all_pairs
    S.D_outlet = edges;
    for (int k = 0; k < O; ++k) {
        for (int i = 0; i < O; i++) {
            if (S.D_outlet[i][k] >= INF) continue;
            for (int j = 0; j < O; j++) {
                if (S.D_outlet[k][j] >= INF) continue;
                int via = S.D_outlet[i][k] + S.D_outlet[k][j];
                if (via < S.D_outlet[i][j]) S.D_outlet[i][j] = via;
            }
        }
    }
    // NOTE: MAYBE SHOULD SWITCH TO JOHNSON'S.. ??
    // FLOYD-WARSHALL IS BETTER FOR DENSE GRAPHS... THIS IS GOING TO BE SPARSE?
}


//just normal A* from a to b but aborts if it exceeds max_cost (battery usage in this context)
int bounded_astar_cost(const Cell& a, const Cell& b, int max_cost) {
    if (!traversable(a) || !traversable(b)) return INF; if (a == b) return 0;
    std::unordered_map<Cell, int, CellHash> g;
    g[a] = 0;
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
    pq.push({octile(a, b), 0, a});
 
    while (!pq.empty()) {
        PQNode top = pq.top(); pq.pop();
        if (top.f > max_cost) return INF;
        auto found_it = g.find(top.c);
        int gc = (found_it == g.end()) ? INF : found_it->second;
        if (top.f - octile(top.c, b) > gc) continue;
        if (top.c == b) return gc;
 
        for (int k = 0; k < 8; k++) {
            int nx = top.c.x + NDX[k], ny = top.c.y + NDY[k];
            Cell n{nx, ny};
            if (!traversable(n)) continue;
            int ng = gc + STEP_COST;
            if (ng > max_cost) continue;

            auto found_it2 = g.find(n);
            if (found_it2 == g.end() || ng < found_it2->second) {
                g[n] = ng;
                pq.push({ng + octile(n, b), 0, n});
            }
        }
    }
    return INF;
}

int reachable_cost(const Cell& a, const Cell& b, int B) {
    int direct = bounded_astar_cost(a, b, B); //direct path w/o going through other cells
 
    int via = INF;
    int bi = idx(b.x, b.y);
    //for each reachable outlet get cost if we go through that outlet
    if (S.nearest_outlet_idx[bi] >= 0) {
        int tail_idx = S.nearest_outlet_idx[bi];
        int tail = S.d_out[bi];
 
        auto from_a = bounded_dijkstra(a, B);
 
        for (int i = 0; i < static_cast<int>(S.outlet_list.size()); ++i) {
            const Cell& oi = S.outlet_list[i];

            int leg1 = from_a[idx(oi.x, oi.y)];
            if (leg1 >= INF) continue;

            int middle = S.D_outlet[i][tail_idx];
            if (middle >= INF) continue;

            long long total = (long long)leg1 + middle + tail;
            if (total < via) via = static_cast<int>(total);
        }
    }
    return std::min(direct, via);
}

//very self explanatory
Cell nearest_reachable_outlet(const Cell& robot, int battery, bool& found) {
    found = false;
    Cell best{-1, -1};
    int  best_d = INF;
    auto from_r = bounded_dijkstra(robot, battery);
    for (const Cell& o : S.outlet_list) {
        int d = from_r[idx(o.x, o.y)];
        if (d < best_d) {best_d = d; best = o; found = true;}
    }
    return best;
}



//this is NOT 3D A*!! - it's just A* that tracks battery alongside and breaks ties by higher remaining battery
std::vector<Cell> astar_with_battery(const Cell& start, const Cell& goal, int start_battery) {
    std::vector<Cell> path;
    if (!traversable(start) || !traversable(goal)) return path;
    std::unordered_map<Cell, int, CellHash>  g;
    std::unordered_map<Cell, int, CellHash>  batt;
    std::unordered_map<Cell, Cell, CellHash> parent;
 
    g[start] = 0;
    batt[start] = start_battery;
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
    pq.push({octile(start, goal), -start_battery, start});
 
    //basic A* loop
    while (!pq.empty()) {
        PQNode top = pq.top(); pq.pop();
        Cell c = top.c;
        if (c == goal) {
            for (Cell cur = goal;;) {
                path.push_back(cur);
                auto it = parent.find(cur);
                if (it == parent.end()) break;
                cur = it->second;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        //also store battery
        int gc = g[c];
        int bc = batt[c];
 
        for (int k = 0; k < 8; ++k) {
            int nx = c.x + NDX[k], ny = c.y + NDY[k];
            Cell n{nx, ny};
            if (!traversable(n)) continue;
            if (bc < STEP_COST) continue; //realistically just checking if 0
 
            int nb = bc - STEP_COST;
            if (S.known_outlets.count(n)) nb = MAX_BATTERY;
            int ng = gc + STEP_COST;
 
            auto itg = g.find(n);
            auto itb = batt.find(n);
            bool accept = false;
            if (itg == g.end()) accept = true;
            //weird structuring of normal A* check, asks if is better than prev g
            else if (ng < itg->second) accept = true;
            else if (ng == itg->second && nb > itb->second) accept = true;
 
            if (accept) {
                g[n] = ng;
                batt[n] = nb;
                parent[n] = c;
                pq.push({ng + octile(n, goal), -nb, n});
            }
        }
    }
    return path;
}

//super simple helper, just checking for # of non-sensed cells around a cell
int information_gain(const Cell& f) {
int count = 0;
    for (int dx = -SENSOR_RAD; dx <= SENSOR_RAD; ++dx) {
        for (int dy = -SENSOR_RAD; dy <= SENSOR_RAD; ++dy) {
            int x = f.x + dx, y = f.y + dy;
            if (in_bounds(x, y) && !S.sensed[idx(x, y)]) ++count;
        }
    }
    return count;
}

/* might switch this to tracking all frontier candidate nodes instead of recomputing,
   will see @ runtime */
std::vector<Cell> compute_exploration_candidates(){
    std::vector<Cell> result;
    std::vector<char> candidate(S.x_size * S.y_size, 0);
    for (int y = 1; y <= S.y_size; y++) {
        for (int x = 1; x <= S.x_size; x++) {
            if (S.sensed[idx(x, y)]) continue;
            for (int dx = -SENSOR_RAD; dx <= SENSOR_RAD; ++dx) {
                for (int dy = -SENSOR_RAD; dy <= SENSOR_RAD; ++dy) {
                    int cx = x + dx, cy = y + dy;
                    if (in_bounds(cx, cy) && traversable(cx, cy)) {
                        candidate[idx(cx, cy)] = 1;
                    }
                }
            }
        }
    }
    for (int y = 1; y <= S.y_size; ++y) {
        for (int x = 1; x <= S.x_size; ++x) {
            if (candidate[idx(x, y)]) result.push_back({x, y});
        }
    }
    return result;
}


//rn doing this explicitly instead of embedding in multigoal A* seach... maybe change later
Cell select_best_exploration_target(const Cell& robot, int battery, bool& found){
    //grab candidates - what are these?
    // any traversable cell with positive gain where gain is # of unsensed cellsaround it
    auto candidates = compute_exploration_candidates();
    found = false;
    Cell best{-1, -1};
    double best_score = -1.0;

    /* MAKING THIS A VARIABLE SO THAT WE CAN CHANGE IT LATER */
    /*LIKELY NOT NEEDED, REMOVE LATER*/
    /////constexpr int RETREAT_MARGIN = 1;

    //precomputing cost from robot to all cells don't know why we were checking A* for each candidate earlier
    auto from_robot = bounded_dijkstra(robot, battery);


    for (const Cell& f: candidates){
        if (f == robot) continue;

        int f_i = idx(f.x, f.y);
        int direct = from_robot[f_i]; //direct cost
        int retreat = S.d_out[f_i]; //cost from f to nearest outlet

        int via_outlet_cost = INF;
        int via_outlet_arrival = -1; //arrival battery!
        //checking the reachable cost but w precomputed stuff mostly
        if (S.nearest_outlet_idx[f_i] >= 0){
            int tail_idx = S.nearest_outlet_idx[f_i];
            int tail = S.d_out[f_i];
            for (int i = 0; i < static_cast<int>(S.outlet_list.size()); i++){
                const Cell& oi = S.outlet_list[i];
                int leg1 = from_robot[idx(oi.x, oi.y)];
                if (leg1 >= INF) continue;
                int middle = S.D_outlet[i][tail_idx];
                if (middle >= INF) continue;
                long long total = (long long)leg1 + middle + tail;
                if (total < via_outlet_cost) via_outlet_cost = (int)total;
            }
            if (via_outlet_cost < INF) via_outlet_arrival = MAX_BATTERY - tail;
        }


        //////
        int cost;
        int arrival_battery;
        //CASE 1: direct path
        //would be weird if direct were ever more than via outlet but just in case
        if (direct < INF && direct <= via_outlet_cost){
            cost = direct;
            arrival_battery = battery - direct;
        }
        //CASE 2: go through outlet network
        else if (via_outlet_cost < INF){
            cost = via_outlet_cost;
            arrival_battery = via_outlet_arrival;
        }
        //CASE 3: not reachable
        else {
            continue;
        }
        if (cost == 0) continue;


        ///actually checking if can retreat
        if (retreat >= INF) continue;


        //exploration budget after you arrive at cell
        int exploration_budget = arrival_battery - retreat;
        if (exploration_budget < 0) exploration_budget = 0;
        //trying to bias towards being able to explore more after arrival, hopefully not outweighting nearest
        double lookahead_factor = 1.0 + static_cast<double>(exploration_budget) / MAX_BATTERY;

        int gain = information_gain(f);
        if (gain == 0) continue;

        double score = (static_cast<double>(gain) * lookahead_factor) / cost; //should i add prox back ??? 

        if (score > best_score){
            best_score = score;
            best = f;
            found = true;
        }
    }
    return best;
}



void next_action_from_plan(int rx, int ry, int* action_ptr) {
    action_ptr[0] = rx;
    action_ptr[1] = ry;
 
    while (!S.current_plan.empty() &&
        S.current_plan.front().x == rx &&
        S.current_plan.front().y == ry) {
        S.current_plan.erase(S.current_plan.begin());
    }
    if (S.current_plan.empty()) return;
 
    const Cell& next = S.current_plan.front();
    action_ptr[0] = next.x;
    action_ptr[1] = next.y;
}


//plan stuff

bool plan_is_invalid(const Cell& robot){
    if (!S.plan_valid || S.current_plan.empty()){
        return true;
    }
    //just making sure it aligns
    const Cell& head = S.current_plan.front(); 
    if (std::abs(head.x - robot.x) > 1 || std::abs(head.y - robot.y) > 1) return true;
    return false;
}

void set_plan(std::vector<Cell> plan, const Cell& target) {
    S.current_plan = std::move(plan);
    S.plan_target = target;
    S.plan_valid = !S.current_plan.empty();
}

/* --------

MAIN PLANNING LOOP!

-----------*/
/* NOTES:
right now it should automatically plan to nearby frontier cells 
so this SHOULD handle the balancing of battery and greedily searching new space,
however if the planner is currently too slow I think we should not replan when
out unless we see a new outlet, then just greedily explore the cells and
track battery/path for return (this can be as simple as just looking at the
len of the path leading towards our cur frontier cell and assuming we take that
back??)
*/
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
    ////// initializing things
    if (!S.initialized || S.x_size != x_size || S.y_size != y_size || S.map_copy_src != map) {
        S.x_size = x_size;
        S.y_size = y_size;
        int N = x_size * y_size;
        S.static_map.assign(N, 0);
        std::memcpy(S.static_map.data(), map, sizeof(int) * N);
        S.map_copy_src = map;
        S.sensed.assign(N, 0);
        S.d_out.assign(N, INF);
        S.nearest_outlet_idx.assign(N, -1);
        S.known_outlets.clear();
        S.last_outlet_set.clear();
        S.outlet_list.clear();
        S.D_outlet.clear();
        S.current_plan.clear();
        S.plan_target = {-1, -1};
        S.plan_valid  = false;
        S.initialized = true;
    }

    //_______SECTION 1: just updating info based on new info ________________

    //updating sensed
    for (int dx = -SENSOR_RAD; dx <= SENSOR_RAD; dx++){
        for (int dy = -SENSOR_RAD; dy <= SENSOR_RAD; dy++){
            int x = robotposeX+dx, y = robotposeY+dy;
            if (in_bounds(x, y)) S.sensed[idx(x, y)] = 1;
        }
    }
    //updating outlets
    for (int i = 0; i < num_visible_chargers; i++){
        Cell o{visible_charger_x[i], visible_charger_y[i]};
        if (in_bounds(o)) S.known_outlets.insert(o);
    }


    //_________SECTION 2: updating outlet path knowledge _____________
    //we're keeping track of shortest paths between outlets but these may update
    if (S.known_outlets.size() != S.last_outlet_set.size()) {
        recompute_outlet_structures();
        S.last_outlet_set = S.known_outlets;
        //need to replan if new outlet discovered
        S.current_plan.clear();
        S.plan_target = {-1, -1};
        S.plan_valid  = false;
    }

    Cell robot{robotposeX, robotposeY};
    Cell goal {goalposeX,  goalposeY};
    int battery = current_charge;
    //recharging if on outlet
    if (S.known_outlets.count(robot)) battery = MAX_BATTERY;

    //_________SECTION 3: goal is reachable path____________________
    int goal_cost = reachable_cost(robot, goal, battery);
    if (goal_cost < INF){
        if (S.plan_target != goal || plan_is_invalid(robot)){ //change strats!
            auto plan = astar_with_battery(robot, goal, battery);
            S.current_plan = plan; //can i use std::move here? signal to compiler ?
            S.plan_target = goal;
            S.plan_valid = !S.current_plan.empty();
        }
        next_action_from_plan(robotposeX, robotposeY, action_ptr);
        return;
    }

    //______SECTION 4: EXPLORE! ____________
    bool found = false;
    // the meat and gravy of it all! picking best target to navigate to.
    Cell target = select_best_exploration_target(robot, battery, found);

    if (!found) {
        //retreat
        bool ok = false;
        Cell fallback = nearest_reachable_outlet(robot, battery, ok);
        if (!ok || fallback == robot) { //already there ? 
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
        //plan back
        if (S.plan_target != fallback || plan_is_invalid(robot)) {
            auto plan = astar_with_battery(robot, fallback, battery);
            set_plan(std::move(plan), fallback);
        }
    }
    else {
        if (S.plan_target != target || plan_is_invalid(robot)) {
            auto plan = astar_with_battery(robot, target, battery);
            set_plan(std::move(plan), target);
        }
    }

    next_action_from_plan(robotposeX, robotposeY, action_ptr);
}
