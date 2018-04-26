#include <math.h>

#include <ctime>
#include "boids.h"

#define PI 3.14159265

float Boids::distance(vector3 pos1,vector3 pos2){
	float c;
	c=(float)sqrt((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y)+(pos1.z-pos2.z)*(pos1.z-pos2.z));
	
	return c;
};

Boids::Boids(vector3 vec,vector3 pos){
	position = vec;
	velocity = pos;
	
}