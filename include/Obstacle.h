// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height
#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle {
public:

	Obstacle();
	Obstacle(double x_in, double y_in, double average_z_in, double width_in, 
                            double length_in);
	
	double x;
	double y;
             double average_z;
	double width;
	double length;
};

#endif /* OBSTACLE_H */