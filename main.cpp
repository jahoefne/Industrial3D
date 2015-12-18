#include "PointCloud.h"
#include "K3DTree.h"

#include <GL/glew.h>
#include "Algorithms.h"
#ifdef __APPLE__

#include <GLUT/glut.h>

#elif defined _WIN32 || defined _WIN64 || defined __linux__
#include <GL/glut.h>
#endif

vector<Point3D>* drawPoints;

std::vector<PointCloud *> clouds; // all visible point clouds
int height = 786, width = 1024, xposStart = 0, yposStart = 0;

/**
 * The display function - gets called by OpenGL to draw to the screen
 */
void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    // draw best fit line
    glLineWidth(2.5);
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glVertex3d(drawPoints->at(0).x, drawPoints->at(0).y, drawPoints->at(0).z);
    glVertex3d(drawPoints->at(1).x, drawPoints->at(1).y, drawPoints->at(1).z);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(drawPoints->at(2).x, drawPoints->at(2).y, drawPoints->at(2).z);
    glVertex3d(drawPoints->at(3).x, drawPoints->at(3).y, drawPoints->at(3).z);
    glEnd();
    //draw best fit plane
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0, 0.5, 1.0, 0.3);
    glBegin(GL_QUADS);
    glVertex3d(drawPoints->at(0).x, drawPoints->at(0).y, drawPoints->at(0).z);
    glVertex3d(drawPoints->at(2).x, drawPoints->at(2).y, drawPoints->at(2).z);
    glVertex3d(drawPoints->at(1).x, drawPoints->at(1).y, drawPoints->at(1).z);
    glVertex3d(drawPoints->at(3).x, drawPoints->at(3).y, drawPoints->at(3).z);
    glEnd();

    glBegin(GL_POINTS);

    for_each(clouds.begin(), clouds.end(), [](PointCloud *cloud) { // draw all clouds not just one
        for (unsigned int i = 0; i < cloud->points.size(); ++i) {
            Point3D pt = cloud->points[i];
          //  printf("Color: %f %f %f",pt.r,pt.g,pt.b);
            if (!pt.ignore){
                glColor3f(pt.r, pt.g, pt.b);
                glPointSize(pt.size);
                glVertex3d(pt.x, pt.y, pt.z);
            }

        }
    });
    glEnd();
    glutSwapBuffers();
}

void keys(int key, int x, int y) {
    if (key == 27 || key == 'q')
        exit(0);

    if (key == 't') {
        clouds.front()->thinning(0.1);
        //clouds.front()->alignTo(clouds.back());
        glutPostRedisplay();
    }
}

void mouse(int button, int state, int x, int y) {
    if (button == 0 && state == GLUT_DOWN) {
        xposStart = x;
        yposStart = y;
    }
}

/**
 * Gets called when the mouse is dragged around the screen
 */
void mouseDrag(int xpos, int ypos) {
    double angleY = (double) (xpos - xposStart) / width * 180.0;
    double angleX = (double) (ypos - yposStart) / height * 180.0;

    for_each(clouds.begin(), clouds.end(), [&](PointCloud *cloud) { // rotate all clouds not just one
        glTranslated(cloud->center.x, cloud->center.y, cloud->center.z);
        glRotated(angleX, -1, 0, 0);
        glRotated(angleY, 0, -1, 0);
        glTranslated(-cloud->center.x, -cloud->center.y, -cloud->center.z);
    });
    //the current mouse position is the start position for the next movement
    xposStart = xpos;
    yposStart = ypos;
    glutPostRedisplay();
}


/**
 * Initializes the camera
 */
void setupCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const double aspect(width / height);

    // To support multiple pointclouds in one view we need to find the biggest scene radius
    const double range = (*max_element(clouds.begin(), clouds.end(),
                                       [](PointCloud *x, PointCloud *y) { // Lambda to find the biggest sceneRadius
                                           return x->sceneRadius < y->sceneRadius;
                                       }))->sceneRadius;

    if (width <= height) glOrtho(-range, range, -range / aspect, range / aspect, 0.0001, 1000);
    else glOrtho(-range * aspect, range * aspect, -range, range, 0.0001, 1000);
    glMatrixMode(GL_MODELVIEW);
}


int main(int argc, char **argv) {
    PointCloud *initialCloud = new PointCloud();
    initialCloud->loadPointsFromFile("../data/stbunny.xyz", 1, 1, 0);
    //initialCloud = initialCloud->smooth(.01);
    drawPoints = computeBestFits(initialCloud->points);

    initialCloud->thinning(0.004);
  //  initialCloud->computeBBox(initialCloud->points);

   //   PointCloud *cloud2 = new PointCloud();
 //   cloud2 = initialCloud->smooth(.01);
  //  PointCloud *cloud2 = new PointCloud();
  //    initialCloud->thinning(.2);
    // initialCloud = initialCloud->smooth(1);
  //  PointCloud *cloud2 = new PointCloud();
  //  cloud2 = initialCloud->smooth(2);

    //cloud2->loadPointsFromFile("../data/angel2.xyz", 1, 0, 0);
    //cloud2->translate(new Point3D(1.0, 1.0, 1.0));

    /*
    Point3D *pt =  &initialCloud->points.front();
    printf("\nSearch for: %lf, %lf, %lf", pt->x,pt->y,pt->z);
    pt = &cloud2->points.front();
    printf("\nSearch for: %lf, %lf, %lf", pt->x,pt->y,pt->z);
    */

    /** Neighbour test */
    // Point3D *pt = new Point3D(-1.79, 0.8, 1.381);
    //   Point3D *pt =  &initialCloud->points.front();
    //   pt->highlight();
/*
    Point3D *closest = initialCloud->kdTree->closestNeighbour(pt);
    closest->highlight();
    printf("\nSearch for: %lf, %lf, %lf", pt->x,pt->y,pt->z);
    printf("\nFound: %lf, %lf, %lf", closest->x,closest->y,closest->z);
*/
    /*   Point3D *max = new Point3D(pt->x, pt->y, pt->z);
       max->translate(new Point3D(0.8, 0.8, 0.8));
       Point3D *min = new Point3D(pt->x, pt->y, pt->z);
       min->translate(new Point3D(-0.8, -0.8, -0.8));
   */
    // std::vector<Point3D *> *neighbors = initialCloud->kdTree->findRange(min, max);
    // std::vector<Point3D *> *neighbors = initialCloud->kdTree->findRadiusNeighbors(pt,0.5);

    /* printf("Neighbors %lu", neighbors->size());
     for_each(neighbors->begin(), neighbors->end(), [](Point3D *pt) {
         pt->highlight();
     });
 */

      clouds.push_back(initialCloud);
    //  clouds.push_back(cloud2);
     // clouds.push_back(cloud3);

    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width, height);

    glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - width) / 2,
                           (glutGet(GLUT_SCREEN_HEIGHT) - height) / 2);

    glutCreateWindow("Industrial 3D Scanning");
    setupCamera();

    glutDisplayFunc(display);

    glutMouseFunc(mouse);
    glutMotionFunc(mouseDrag);
    glutSpecialFunc(keys);

    // glewInit();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslated(-initialCloud->center.x, -initialCloud->center.y, -initialCloud->center.z - initialCloud->sceneRadius);
    glutPostRedisplay();
    glutMainLoop();
    return 0;
}
