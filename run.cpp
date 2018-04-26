#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include "boids.h"

#define BOID_COUNT 10
#define MIN_DISTANCE 0.3
#define MAX_DISTANCE 1
#define RULE1_FACTOR 100
#define RULE2_FACTOR 1
#define RULE3_FACTOR 8
#define MIN_VELOCITY 0.1
#define MAX_VELOCITY 0.9
// globals of program
Boids* boid_list[BOID_COUNT];
int oldTimeSinceStart = 0;
vector3 summable = {0,0,0};
// for rule one
vector3 tem = {0.0,0.0,0.0};
std::vector<int> closestNeighbours;

// for rule three
vector3 tem3 = {0.0,0.0,0.0};
// for rule two
vector3 tem2  = {0,0,0};

void myDisplay();
void myInit()
{
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();
	gluPerspective(60.0, 64/48.0, 0.1, 100);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();
	gluLookAt(0,0, 5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glColor3d(0,0,0);
};

void obstacle(){
	for (int i = 0;i<BOID_COUNT;i++){

		if((boid_list[i]->position.y < -WIN_HEIGHT)||(boid_list[i]->position.y > WIN_HEIGHT)){
			boid_list[i]->velocity.y = -(boid_list[i]->velocity.y);
		}
		if((boid_list[i]->position.x > WIN_WIDTH )||(boid_list[i]->position.x < -WIN_WIDTH )) {
			boid_list[i]->velocity.x = -boid_list[i]->velocity.x;

		}
	}
};
int k =0;
void method1(int i){

	closestNeighbours.clear();
	tem.x = 0.0;
	tem.y = 0.0;
	tem.z = 0.0;
	float temp;
	for(int j = 0;j< BOID_COUNT;j++){
		if(i!=j){
			temp = Boids::distance(boid_list[i]->position,boid_list[j]->position);
			if((temp <MAX_DISTANCE)&&(temp>MIN_DISTANCE)){
				closestNeighbours.push_back(j);
			}
		}
		
	}

	for(int j = 0; j<closestNeighbours.size();j++){
		tem.x = tem.x + boid_list[closestNeighbours[j]]->position.x;
		tem.y = tem.y + boid_list[closestNeighbours[j]]->position.y;
		tem.z = tem.z + boid_list[closestNeighbours[j]]->position.z;
	}
	if(closestNeighbours.size()!=0){
		tem.x = (tem.x/closestNeighbours.size())-boid_list[i]->position.x;
		tem.y = (tem.y/closestNeighbours.size())-boid_list[i]->position.y;	
		tem.z = (tem.z/closestNeighbours.size())-boid_list[i]->position.z;
	}	
	tem.x = tem.x/RULE1_FACTOR;
	tem.y = tem.y/RULE1_FACTOR;
	tem.z = tem.z/RULE1_FACTOR;


}
void method2(int i){
	closestNeighbours.clear();
	tem2.x = 0.0;
	tem2.y = 0.0;
	tem2.z = 0.0;
	float temp;
	for(int j = 0;j< BOID_COUNT;j++){
		if(i!=j){
			temp = Boids::distance(boid_list[i]->position,boid_list[j]->position);
			if((temp<=MIN_DISTANCE)){
				closestNeighbours.push_back(j);
			}
		}
	}
	for(int j = 0; j<closestNeighbours.size();j++){
		tem2.x = tem2.x + boid_list[closestNeighbours[j]]->position.x-boid_list[i]->position.x;
		tem2.y = tem2.y + boid_list[closestNeighbours[j]]->position.y-boid_list[i]->position.y;
		tem2.z = tem2.z + boid_list[closestNeighbours[j]]->position.z-boid_list[i]->position.z;
	}
	tem2.x = -tem2.x/RULE2_FACTOR;
	tem2.y = -tem2.y/RULE2_FACTOR;
	tem2.z = -tem2.z/RULE2_FACTOR;
}
void method3(int i){
	closestNeighbours.clear();
	tem3.x = 0.0;
	tem3.y = 0.0;
	tem3.z = 0.0;
	float temp;
	for(int j = 0;j< BOID_COUNT;j++){
		if(i!=j){
			temp = Boids::distance(boid_list[i]->position,boid_list[j]->position);
			if((temp <MAX_DISTANCE)&&(temp>MIN_DISTANCE)){
				closestNeighbours.push_back(j);
			}
		}
		
	}
	for(int j = 0; j<closestNeighbours.size();j++){
		tem3.x = tem3.x + boid_list[closestNeighbours[j]]->velocity.x;
		tem3.y = tem3.y + boid_list[closestNeighbours[j]]->velocity.y;
		tem3.z = tem3.z + boid_list[closestNeighbours[j]]->velocity.z;
	}
	if(closestNeighbours.size()!=0){
		tem3.x = (tem3.x/closestNeighbours.size())-boid_list[i]->velocity.x;
		tem3.y = (tem3.y/closestNeighbours.size())-boid_list[i]->velocity.y;	
		tem3.z = (tem3.z/closestNeighbours.size())-boid_list[i]->velocity.z;
	}	
	tem3.x = tem3.x/RULE3_FACTOR;
	tem3.y = tem3.y/RULE3_FACTOR;
	tem3.z = tem3.z/RULE3_FACTOR;

}
void calculatePositions(float frameTime){

	for(int i =0;i<BOID_COUNT;i++){
		method1(i);
		method2(i);
		method3(i);
		boid_list[i]->velocity.x =  boid_list[i]->velocity.x + tem.x + tem3.x + tem2.x;
		boid_list[i]->velocity.y =  boid_list[i]->velocity.y + tem.y + tem3.y + tem2.y; 

		if(boid_list[i]->velocity.x >MAX_VELOCITY ||boid_list[i]->velocity.x <-MAX_VELOCITY  ){
			if(boid_list[i]->velocity.x>0)
				boid_list[i]->velocity.x =boid_list[i]->velocity.x/2 ;
			else
				boid_list[i]->velocity.x = -boid_list[i]->velocity.x/2 ;
		}
		if(boid_list[i]->velocity.y >MAX_VELOCITY ||boid_list[i]->velocity.x <-MAX_VELOCITY ){
			if(boid_list[i]->velocity.y>0)
				boid_list[i]->velocity.y = MIN_VELOCITY;
			else
				boid_list[i]->velocity.y = -MIN_VELOCITY; 
		}
		
		if(boid_list[i]->velocity.x <MIN_VELOCITY||boid_list[i]->velocity.x >-MIN_VELOCITY ){
		 	if(boid_list[i]->velocity.x>0)
				boid_list[i]->velocity.x = MAX_VELOCITY/2;
			else
				boid_list[i]->velocity.x = -MAX_VELOCITY/2; 
		}
		if(boid_list[i]->velocity.y <MIN_VELOCITY||boid_list[i]->velocity.y >-MIN_VELOCITY){
		 	if(boid_list[i]->velocity.y>0)
				boid_list[i]->velocity.y = MAX_VELOCITY/2;
			else
				boid_list[i]->velocity.y = -MAX_VELOCITY/2; 
		}
		//boid_list[i]->velocity.z = boid_list[i]->velocity.z + tem.y;
		//method 2


	}


	for(int i = 0; i<BOID_COUNT;i++){
			boid_list[i]->position.x=boid_list[i]->position.x+boid_list[i]->velocity.x*frameTime;
			boid_list[i]->position.y=boid_list[i]->position.y+boid_list[i]->velocity.y*frameTime;
	}
	obstacle();
	if(k<200){
		for(int i = 0;i<BOID_COUNT;i++)
		std::cout << boid_list[i]->position.x<<" "<<boid_list[i]->position.y<<boid_list[i]->position.z<<std::endl;
		k++;
	}
	
};



void myIdle()
{
    int timeSinceStart = glutGet(GLUT_ELAPSED_TIME);
    int deltaTime = timeSinceStart - oldTimeSinceStart;
	calculatePositions((float)deltaTime/1000.0);
	myDisplay();
    oldTimeSinceStart = timeSinceStart;
 
};


void myDisplay(void)
{
 
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_LINE_LOOP); // Draw Platform
	glVertex3d(WIN_WIDTH,-WIN_HEIGHT,0.5); 
	glVertex3d(WIN_WIDTH,WIN_HEIGHT,0.5);
	glVertex3d(-WIN_WIDTH,WIN_HEIGHT,0.5);
	glVertex3d(-WIN_WIDTH,-WIN_HEIGHT,0.5);
	glEnd();
	// BOIDS
	 // draw and place Sphere
    for(int i =0 ;i< BOID_COUNT;i++){
		glPushMatrix();
	    glTranslated(boid_list[i]->position.x,boid_list[i]->position.y,boid_list[i]->position.z);
        glBegin(GL_TRIANGLES);
            glColor3f(0.5,0.5,0.5);
            glVertex3f(0,0.25,0);
            glVertex3f(-0.08,0,0);
            glVertex3f(0.08,0,0);
        glEnd();
		glPopMatrix();
    }
	
	glutSwapBuffers();
}


int main(int argc, char **argv)
{
	vector3 pos,velo;
	float rx =0,ry=0,vx=0,vy=0;
	srand (static_cast <unsigned> (time(0)));
    for(int i =0;i<BOID_COUNT;i++){
		rx = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*WIN_WIDTH;
		ry = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*WIN_HEIGHT;
        pos.x = rx;
		pos.y = ry;
		pos.z = 0;
		vx = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*VELOCITY;
		vy = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*VELOCITY;
		velo.x = vx;
		velo.y = vy;
		velo.z = 0;
        boid_list[i] = new Boids(pos,velo);
    }
		
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
	glutInitWindowSize(800,600);
	glutInitWindowPosition(50, 50);
	glutCreateWindow("Rolling");
	glutDisplayFunc(myDisplay);
	glutIdleFunc(myIdle);
	glClearColor(1.0f, 1.0f, 1.0f,0.0f);
	glViewport(0, 0, 800, 600);
	//glutKeyboardFunc(myKeyboard);
	myInit();
	glutMainLoop();


	return 0;
}
