// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height

#include "Obstacle.h"

Obstacle::Obstacle(){
    x = -1;
    y = -1;
    width = -1;
    height = -1;
    average_z = -1;
}

Obstacle::Obstacle(int x_in, int y_in, double average_z_in, int width_in, 
                            int height_in){
    x = x_in;
    y = y_in;
    average_z = average_z_in;
    width = width_in;
    height = height_in;
}
