// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height

#include "Obstacle.h"

Obstacle::Obstacle(){
    x = -1;
    y = -1;
    width = -1;
    length = -1;
}

Obstacle::Obstacle(double x_in, double y_in, double average_z_in, double width_in, 
                            double length_in){
    x = x_in;
    y = y_in;
             average_z = average_z_in;
    width = width_in;
    length = length_in;
}
