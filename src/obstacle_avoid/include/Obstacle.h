// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height
#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle {
public:

	Obstacle();
	Obstacle(int x_in, int y_in, int width_in, int length_in);
	
	double x;
	double y;
	double width;
	double length;
};

#endif /* OBSTACLE_H */