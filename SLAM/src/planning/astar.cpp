#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <iostream>
#include <set>
#include <queue>
#include <vector>
#include <algorithm>

class Cell{
public:    
    int c_id, c_x, c_y;
    float G, H, C; // current cost and Heuristic cost
    bool opened, closed; // whether the cell is in openlist and in closedlist
    Cell* parent; // parent cell
    Cell():c_x(0),c_y(0),c_id(0), parent(NULL), G(0), H(0), C(0), opened(false){}    
    Cell(int x, int y, int id)
    :c_x(x),c_y(y),c_id(id), parent(NULL), G(0), H(0), C(0), opened(false){}
    ~Cell(){delete parent;}
    inline float F(){ return G+H+C; }
};

inline float EuclideanDistance(Cell* cell1, Cell* cell2){
    return sqrtf(powf(cell1->c_x - cell2->c_x,2) + powf(cell1->c_y - cell2->c_y,2));
}

struct CellComparator {
    inline bool operator() (Cell* c1, Cell* c2){
        return c1->F() > c2->F();
    }
};

std::vector<int> getNeighbors(int& current_id, const std::vector<Cell*>& cells, int& width, int& height){
    std::vector<int> neighbors;
    int x = cells[current_id]->c_x;
    int y = cells[current_id]->c_y;
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            if(i == 0 && j == 0) continue;
            int x_temp = x + i;
            int y_temp = y + j;
            if((x_temp >= 0 && x_temp < width) && (y_temp >= 0 && y_temp < height)){
                neighbors.push_back(x_temp + width*y_temp);
            }
        }    
    }
    return neighbors;
}

bool ObstacleInPath(int& x0, int& y0, int& x1, int& y1, const ObstacleDistanceGrid& distances, float minDistanceToObstacle){
    float theta = std::atan2(y1-y0, x1-x0);
    float x = x0;
    float y = y0;
    int dir_x = x1 > x0 ? 1:-1;
    int dir_y = y1 > y0 ? 1:-1;  
    int int_x = 0;
    int int_y = 0;  

    while(true){
        x += std::cos(theta);
        y += std::sin(theta);

        int_x = static_cast<int>(x);
        int_y = static_cast<int>(y);

        if(distances(x,y) < 1.05*minDistanceToObstacle){
            return true;
        }

        if( dir_x * (x1 - int_x) <= 0 && dir_y * (y1 - int_y) <= 0 ){
            return false;
        }
    }
}

float ObstacleCost(int& x,int& y,const ObstacleDistanceGrid& distances, const SearchParams& params){
    float cellDistance = distances(x,y);
    if(cellDistance > params.minDistanceToObstacle && cellDistance < params.maxDistanceWithCost){
        return (params.maxDistanceWithCost - cellDistance);
    }
    return 0.0f;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    /* Turning points from global frame to cell frame */
    int start_x = static_cast<int>((start.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter());
    int start_y = static_cast<int>((start.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter());
    int goal_x  = static_cast<int>((goal.x  - distances.originInGlobalFrame().x) * distances.cellsPerMeter());
    int goal_y  = static_cast<int>((goal.y  - distances.originInGlobalFrame().y) * distances.cellsPerMeter());
    int width   = distances.widthInCells();
    int height  = distances.heightInCells();
    int start_id = start_x + start_y * width;
    int goal_id  = goal_x  + goal_y  * width;
    
    /* Path with only the start position: Failed to find a valid path */
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();

    /* Check start position */
    if(!distances.isCellInGrid(start_x, start_y)){
        std::cout << "No path found: Invalid start" << std::endl;
        return path;
    }

    /* Check goal position*/
    if(!distances.isCellInGrid(goal_x, goal_y)
       || distances(goal_x,goal_y) < params.minDistanceToObstacle){
        std::cout << "No path found: Invalid goal "<< std::endl;
        return path;
    }
    
    /* Create data structures for A* algorithm */
    std::priority_queue<Cell, std::vector<Cell*>, CellComparator> openlist;
    // std::set<int> closedlist;
    std::vector<int> successors;
    std::vector<int> searched_path;

    /* Create a cell map in reference to ObstacleDistanceGrid*/   
    std::vector<Cell*> cells;
    int id = 0;
    for(int y = 0; y < height; ++y){
        for(int x = 0; x < width; ++x){
            id = x + y * width;
            cells.push_back(new Cell(x, y, id));
            cells[id]->C = ObstacleCost(x, y, distances, params);
        }
    }

    /* Initialize start cell and push into openlist */
    openlist.push(cells[start_id]);
    cells[start_id]->G = 0;
    cells[start_id]->H = 0;
    cells[start_id]->opened = true;

    /* Start path searching */
    bool reachGoal = false;
    float F_temp = 0.0f;
    int successors_size = 0;

    while(!openlist.empty() && !reachGoal){
        Cell* parent_cell = openlist.top();
        openlist.pop();
        successors = getNeighbors(parent_cell->c_id, cells, width, height);
        successors_size = successors.size();
        for(int i = 0; i< successors_size; i++){
            Cell* successor = cells[successors[i]];
            if(successor->c_id == goal_id){
                /* Reached the goal */ 
                successor->G = parent_cell->G + EuclideanDistance(parent_cell, successor);
                successor->H = 0;
                successor->parent = parent_cell;
                reachGoal = true;
                break;
            }else if(distances(successor->c_x,successor->c_y) < 1.05*params.minDistanceToObstacle){
                /* Reached the cell that is too close to the obstacle */
                continue;
            }else if(successor->opened){
                /* Reached the cell already in openlist */
                F_temp = parent_cell->G 
                    + EuclideanDistance(parent_cell, successor)
                    + EuclideanDistance(cells[goal_id], successor)
                    + successor->C;//ObstacleCost(successor->c_x,successor->c_y, distances,params);
                if(F_temp > successor->F()) continue;
            }else if(successor->closed){
                /* Reached the cell already in closedlist */
                F_temp = parent_cell->G 
                    + EuclideanDistance(parent_cell, successor)
                    + EuclideanDistance(cells[goal_id], successor)
                    + successor->C;//ObstacleCost(parent_cell->c_x,parent_cell->c_y,distances,params);
                if(F_temp > successor->F()) continue;
                else{
                    // closedlist.erase(successor->c_id);
                    successor-> opened = false;
                    successor-> closed = false;
                }  
            }
            /* Update cost and parent cell */
            successor->G = parent_cell->G + EuclideanDistance(parent_cell, successor);
            successor->H = EuclideanDistance(cells[goal_id], successor);
                         //+ ObstacleCost(successor->c_x,successor->c_y, distances,params);
            successor->parent = parent_cell;

            /* Push to openlist if not opened */
            if(!successor->opened){
                openlist.push(successor);
                successor->opened = true;
            }
        }

        /* Push to closedlist */
        // closedlist.insert(parent_cell->c_id);
        parent_cell->closed = true;
    }

    /* Cannot reach goal cell */
    if (cells[goal_id]->parent == NULL){
        std::cout << "No path found: cannot reach goal" << std::endl;
        return path;
    }

    /* Reconstruct the searched path */
    Cell* cptr = cells[goal_id];
    searched_path.push_back(cptr->c_id);
    while(cptr->c_id != start_id){
        cptr = cptr->parent;
        searched_path.push_back(cptr->c_id);
    }
    std::reverse(searched_path.begin(),searched_path.end());
    // closedlist.clear();

    /* Smooth generated path -> Reduce redundant waypoints */
    Cell *pre_cell, *cnt_cell, *post_cell;
    robot_path_t smoothed_searched_path;
    smoothed_searched_path.utime = start.utime;
    smoothed_searched_path.path.push_back(start);
    for(std::size_t i = 1; i < searched_path.size()-1; i++){
        cnt_cell = cells[searched_path[i]];
        pre_cell = cells[searched_path[i-1]];
        post_cell = cells[searched_path[i+1]];
        if(!ObstacleInPath( pre_cell->c_x, pre_cell->c_y, 
                            post_cell->c_x, post_cell->c_y, 
                            distances, 
                            params.minDistanceToObstacle)){
            searched_path.erase(searched_path.begin() + i);
            i--;
        }else{
            pose_xyt_t cnt_pose;
            cnt_pose.utime = start.utime;
            cnt_pose.x = cnt_cell->c_x * distances.metersPerCell() + distances.originInGlobalFrame().x;
            cnt_pose.y = cnt_cell->c_y * distances.metersPerCell() + distances.originInGlobalFrame().y;
            cnt_pose.theta = 0.0f;
            smoothed_searched_path.path.push_back(cnt_pose);
        }
    }
    smoothed_searched_path.path.push_back(goal);
    searched_path.clear();

    /* Return the whole path */
    smoothed_searched_path.path_length = smoothed_searched_path.path.size();
    return smoothed_searched_path;
}
