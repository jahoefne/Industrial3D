#include <stdio.h>
#include <stdlib.h>

#include <iostream>   //for making output text to the console with "cout"
#include "PointCloud.h"
#include "K3DTree.h"

#include <GL/glew.h>
#include <GL/glut.h>


using namespace std;

PointCloud *cloud = new PointCloud();
int height = 768;
int width = 1024;
int xposStart = 0;
int yposStart = 0;

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    glPointSize(1);
    glColor3ub(0, 204, 0);

    glBegin(GL_POINTS);
    for (int i = 0; i < cloud->points.size(); ++i) {
        glVertex3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
    glEnd();

    /*
    glPushMatrix();
    glColor3ub(255, 0, 0);
    glTranslated(cloud->center.x, cloud->center.y, cloud->center.z);
    glutSolidSphere(0.1/cloud->sceneRadius,  20, 30);
    glPopMatrix();
    */
    glutSwapBuffers();
}

void keys(int key, int x, int y) {
    if (key == 27 || key == 'q')
        exit(0);
}

void mouse(int button, int state, int x, int y) {
    if (button == 0 && state == GLUT_DOWN) {
        xposStart = x;
        yposStart = y;
    }
}

void mouseDrag(int xpos, int ypos) {
    double angleY = (double) (xpos - xposStart) / width * 180.0;
    double angleX = (double) (ypos - yposStart) / height * 180.0;

    glTranslated(cloud->center.x, cloud->center.y, cloud->center.z);
    glRotated(angleX, -1, 0, 0);
    glRotated(angleY, 0, -1, 0);
    glTranslated(-cloud->center.x, -cloud->center.y, -cloud->center.z);

    //the current mouse position is the start position for the next movement
    xposStart = xpos;
    yposStart = ypos;
    glutPostRedisplay();
}


void setupCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect(width / height);
    const double range = cloud->sceneRadius;

    if (width <= height) glOrtho(-range, range, -range / aspect, range / aspect, 0.0001, 1000);
    else glOrtho(-range * aspect, range * aspect, -range, range, 0.0001, 1000);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char **argv) {

    cloud->loadPointsFromFile("/Users/moe/Desktop/Industrial3D/data/bunny.xyz");

    K3DTree* tree = new K3DTree(cloud);


    /** Set up glut/glew foo */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width, height);

    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - width) / 2,
                           (glutGet(GLUT_SCREEN_HEIGHT) - height) / 2);

    glutCreateWindow("Industrial 3D Scanning");
    setupCamera();

    printf("sceneRadius %f", cloud->sceneRadius);

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseDrag);
    glutSpecialFunc(keys);

    glewInit();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(-cloud->center.x, -cloud->center.y, -cloud->center.z - cloud->sceneRadius);

    glutMainLoop();
    return EXIT_SUCCESS;
}