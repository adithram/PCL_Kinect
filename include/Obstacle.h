// Obstacle object. Will be published. Contains x location, y location, approximate width, and approximate height
#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle {
public:

	Obstacle();
	Obstacle(int x_in, int y_in, double average_z_in, int width_in, 
                            int height_in);
	
	int x;
	int y;
             double average_z;
	int width;
	int height;
};

#endif /* OBSTACLE_H */