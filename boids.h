#include <stdlib.h>
#include <iostream>
#define WIN_HEIGHT 2.5
#define WIN_WIDTH  3.0
#define VELOCITY 0.5
struct vector3{
    float x,y,z;
};

class Boids{
    public:
        vector3 position;
        vector3 velocity;
    public:
        Boids(vector3 vec,vector3 pos);
	    static float distance(vector3 pos1,vector3 pos2);
        
};
