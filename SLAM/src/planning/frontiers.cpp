#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier, 
                              const pose_xyt_t& pose, 
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);




std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    
    /* Find the centers of the frontiers */
    std::vector<Point<float>> frontier_centers;
    float x,y;
    for(size_t i = 0; i < frontiers.size(); i++){
        x = 0.0f;
        y = 0.0f;
        int frontier_size = frontiers[i].cells.size();
        for(int j = 0; j < frontier_size; j++){
            x += frontiers[i].cells[j].x;
            y += frontiers[i].cells[j].y;
        }
        frontier_centers.push_back(Point<float>(x/frontier_size, y/frontier_size));
    }

    /* If all frontier explored */
    robot_path_t emptyPath;
    emptyPath.utime = robotPose.utime;
    emptyPath.path.push_back(robotPose);    
    emptyPath.path_length = emptyPath.path.size();

    if(frontier_centers.empty()) return emptyPath;
    
    /* Find nearest frontier */
    Point<float> current_position = Point<float>(robotPose.x,robotPose.y);
    int nearest_frontier_idx = 0;
    float nearest_frontier_distance = distance_between_points(frontier_centers[0], current_position);
    for(size_t i = 1; i < frontier_centers.size(); i++){
        if(distance_between_points(frontier_centers[i],current_position) < nearest_frontier_distance){
            nearest_frontier_distance = distance_between_points(frontier_centers[i],current_position);
            nearest_frontier_idx = i;
        }
    }
    Point<float> target_frontier = frontier_centers[nearest_frontier_idx];

    /* Find nearest navigable cell using BFS */
    pose_xyt_t goalPose = nearest_navigable_cell(target_frontier, map, planner);
    //if(goalPose.x == robotPose.x && goalPose.y == robotPose.y) return emptyPath;
    std::cout << "Founded frontier goal pose: " << goalPose.x << " " << goalPose.y << std::endl;
    /* Plan path using A* */
    return planner.planPath(robotPose, goalPose);
}


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}

pose_xyt_t nearest_navigable_cell(Point<float> desiredPosition, 
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner)
{
    ObstacleDistanceGrid grid = planner.obstacleDistances();


    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    Point<int> robotCell = global_position_to_grid_cell(desiredPosition, map);

    // if(grid(robotCell.x, robotCell.y) >= 0.30){
    //     pose_xyt_t disiredPose;
    //     disiredPose.x = desiredPosition.x;
    //     disiredPose.y = desiredPosition.y;
    //     return disiredPose;
    // }

    std::queue<Point<int>> cellQueue;
    std::set<Point<int>> visitedCells;

    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);

    while(!cellQueue.empty()){
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y)){
                continue;
            }else if(grid(neighbor.x, neighbor.y) < 0.28){
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }else{
                pose_xyt_t goalPose;
                goalPose.x = neighbor.x * map.metersPerCell() + map.originInGlobalFrame().x;
                goalPose.y = neighbor.y * map.metersPerCell() + map.originInGlobalFrame().y;
                return goalPose;
            }
        }
    }
    std::cout << "Cannot find nearest cell" << std::endl;
    pose_xyt_t emptyPose;
    emptyPose.x = desiredPosition.x;
    emptyPose.y = desiredPosition.y;
    return emptyPose;
}
