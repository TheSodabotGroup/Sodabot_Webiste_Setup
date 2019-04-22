#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    const int delta_x[] = {1, 1,-1,-1};
    const int delta_y[] = {1,-1, 1,-1};

    int min = 0;
    for(int y = 0; y < height_; ++y){
        for(int x = 0; x < width_; ++x){
            if (map.logOdds(x,y) >= 0) {
                cells_[x + y*width_] = -0.0f;
                continue;
            }

            min = std::sqrt(width_ * height_);
            for(int i = 0; i < width_; i++){
                if(map.logOdds(i,y) >= 0 && std::abs(x-i) <= min){
                    min = std::abs(x-i);
                }
            }
            for(int j = 0; j < height_; j++){
                if(map.logOdds(x,j) >= 0 && std::abs(y-j) <= min){
                    min = std::abs(y-j);
                }
            }
            for(int k = 0; k < 4; k++){
                int tmp_x = x + delta_x[k];
                int tmp_y = y + delta_y[k];
                while(map.isCellInGrid(tmp_x,tmp_y)){
                    if(map.logOdds(tmp_x, tmp_y) >= 0 && std::sqrt(2) * std::abs(tmp_x-x) <= min){
                        min = std::sqrt(2) * std::abs(tmp_x-x);
                    }
                    tmp_x += delta_x[k];
                    tmp_y += delta_y[k];
                }
            }
            cells_[x + y*width_] = min * map.metersPerCell();
        }
    }


}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
