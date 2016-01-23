#include "PointCloud.h"
#include "K3DTree.h"

#include <GL/glew.h>
#include "Algorithms.h"
#include "SVD.h"
#ifdef __APPLE__

#include <GLUT/glut.h>

#elif defined _WIN32 || defined _WIN64 || defined __linux__
#include <GL/glut.h>
#endif

PointCloud *initialCloud;
vector<Point3D>* drawPoints;
std::vector<Point3D> normals;
bool drawBestFitLine = false;
bool drawBestFitPlane= false;
Point3D spherecenter;
double sphereRadius = 0;
bool drawBestFitSphere = false;
std::vector<PointCloud *> clouds; // all visible point clouds
int height = 786, width = 1024, xposStart = 0, yposStart = 0;


/////////////////////////////////////////////////////////////////////////

GLuint program;

static const GLchar * vertex_shader[] ={"\
varying vec3 N;\
varying vec3 v;  \
varying vec4 FrontColor;  \
void main(void)     \
{                 \
  v = vec3(gl_ModelViewMatrix * gl_Vertex);   \
  N = normalize(gl_NormalMatrix * gl_Normal);   \
  FrontColor = gl_Color;                       \
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;  \
}"};

static const GLchar * fragment_shader[] ={"\
        varying vec3 N;\
        varying vec3 v;\
        varying vec4 FrontColor;\
        void main(void)\
        { \
            vec3 lightPosition=vec3(1.0,1.0,1.0);        \
            vec4 ambientColor=vec4(0.1, 0.1, 0.1, 1.0);    \
            vec4 specularColor=vec4(0.7, 0.7, 0.7, 1.0);  \
            float shininess=100.0;                       \
            vec3 N = normalize(N);                       \
            vec3 L = normalize(lightPosition - v);        \
            vec3 E = normalize(-v);                      \
            vec3 R = normalize(-reflect(L, N));          \
            vec4 Iamb = ambientColor;                     \
            vec4 Idiff = FrontColor * max(abs(dot(N, L)), 0.0);  \
            Idiff = clamp(Idiff, 0.0, 1.0);             \
            vec4 Ispec = specularColor * pow(max(dot(R, E), 0.0), 0.03* shininess);\
            Ispec = clamp(Ispec, 0.0, 1.0);\
            gl_FragColor = Iamb + Idiff + Ispec;\
        }"};



void create_shaders()
{
    GLuint v, f;

    v = glCreateShader(GL_VERTEX_SHADER);
    f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(v, 1, vertex_shader, NULL);
    glShaderSource(f, 1, fragment_shader, NULL);
    glCompileShader(v);
    GLint compiled;
    glGetShaderiv(v, GL_COMPILE_STATUS, &compiled );
    if ( !compiled ) {
//        GLsizei  maxLength, length;
//        glGetShaderiv( v, GL_INFO_LOG_LENGTH, &maxLength );
//        GLchar* log = malloc(sizeof(GLchar)*(maxLength+1));
//        glGetShaderInfoLog(v,  maxLength, &length, log);
        printf("Vertex Shader compilation failed: %s\n");
//        free(log);
    }
    glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &compiled );
    if ( !compiled ) {
//        GLsizei  maxLength, length;
//        glGetShaderiv( f, GL_INFO_LOG_LENGTH, &maxLength );
//        GLchar* log = malloc(sizeof(GLchar)*(maxLength+1));
//        glGetShaderInfoLog(f,  maxLength, &length, log);
        printf("Fragment Shader compilation failed: %s\n");
//        free(log);
    }
    program = glCreateProgram();
    glAttachShader(program, f);
    glAttachShader(program, v);
    glLinkProgram(program);
    GLint linked;
    glGetProgramiv(program, GL_LINK_STATUS, &linked );
    if ( !linked ) {
//        GLsizei len;
//        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len );
//        GLchar* log = malloc(sizeof(GLchar)*(len+1));
//        glGetProgramInfoLog(program, len, &len, log );
        printf("Shader linking failed: %s\n");
//        free(log);
    }
    glUseProgram(program);
}


////////////////////////////////////////////////////////////////////////

void computeNormalVectors(std::vector<Point3D>& points, double radius, std::vector<Point3D>& normals)
{

    Point3D* rangeBegin = points.data();                          //C++-Style to get address of first element
    Point3D* rangeEnd = rangeBegin + points.size();               //compute address of last element (exclusive)

    //  const KDNode* rootNode = buildKdTree(rangeBegin, rangeEnd, 0);  //recusrsively build kdTree for given range, starting with tree depth = 0
    K3DTree* kdTree1 = new K3DTree(&points); // Init Kd-Tree

    normals.resize(points.size());  //make sure that array for the normal vectors has the same size as we have points

    //because all the relevant variables are declared inside the for loop we can safely parallelize the code
    //without having races between different parallel omp threads.
#pragma omp parallel for
    for (ptrdiff_t i = 0; i < (ptrdiff_t)points.size(); ++i)
    {
        vector<Point3D *> neighbors;
        std::vector<Point3D> neighbors2;
        neighbors = *(kdTree1->findRadiusNeighbors(&(points[i]),radius));


        //      findNeighbors(rootNode, points[i], radius, neighbors, 0); //find localneighbor points within given radius

        if (neighbors.size() > 2) //best-fit plane needs at least 3 points
        {
            Matrix M(3, 3);
            //
            long Numpoints = neighbors.size();
            for(int n = 0; n < Numpoints; n++)
            {
                Point3D *  point1;
                point1 = neighbors[n];
                Point3D  point11;
                point11.x = point1->x;
                point11.y = point1->y;
                point11.z = point1->z;
                neighbors2.push_back(point11);
            }
            //
           computeCoarianceMatrix(neighbors2, M);

            SVD::computeSymmetricEigenvectors(M);

            Point3D normal = Point3D(M(0, 2), M(1, 2), M(2, 2)); //assign normal vector of the plane (eigenvector corresponding to smalles eigenvalue == direction of smallest variance)
            normals[i] = normal;
        }
        else //if there are not enough points in the neighborhood we assign an empty/useless normal
            normals[i] = Point3D(0, 0, 0);
    }
}


/**
 * The display function - gets called by OpenGL to draw to the screen
 */
void display(void) {


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);

    // draw best fit line

    glLineWidth(2.5);
    if (drawBestFitLine)
    {
        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3d(drawPoints->at(0).x, drawPoints->at(0).y, drawPoints->at(0).z);
        glVertex3d(drawPoints->at(1).x, drawPoints->at(1).y, drawPoints->at(1).z);
        glEnd();
    }
    if (drawBestFitPlane)
    {
        //draw best fit plane
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3d(drawPoints->at(0).x, drawPoints->at(0).y, drawPoints->at(0).z);
        glVertex3d(drawPoints->at(1).x, drawPoints->at(1).y, drawPoints->at(1).z);
        glVertex3d(drawPoints->at(2).x, drawPoints->at(2).y, drawPoints->at(2).z);
        glVertex3d(drawPoints->at(3).x, drawPoints->at(3).y, drawPoints->at(3).z);
        glEnd();
    }

    if (drawBestFitSphere)
    {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glPushMatrix();
        glColor3ub(155, 255, 0);
        glTranslated(spherecenter.x, spherecenter.y, spherecenter.z );
        GLUquadric* quad = gluNewQuadric();
        gluSphere(quad, sphereRadius, 30, 30);
        gluDeleteQuadric(quad);
        glPopMatrix();

        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glEnd();
    }




    glBegin(GL_POINTS);

    for_each(clouds.begin(), clouds.end(), [](PointCloud *cloud) { // draw all clouds not just one
        for (unsigned int i = 0; i < cloud->points.size(); ++i) {
            Point3D pt = cloud->points[i];
          //  printf("Color: %f %f %f",pt.r,pt.g,pt.b);
            if (!pt.ignore){
                glColor3f(pt.r, pt.g, pt.b);
                glPointSize(3.0f);
                glNormal3f(normals[i].x,normals[i].y,normals[i].z);
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
    initialCloud = new PointCloud();
    initialCloud->loadPointsFromFile("../data/horse.xyz", 1, 1, 0);
    //initialCloud = initialCloud->smooth(.01);
    //drawPoints = computeBestFits(initialCloud->points);


   // drawPoints = computeBestFitPlane(initialCloud->points);





    double radius = 0.001; //for the horse, buddha, dragon

    computeNormalVectors(initialCloud->points, radius, normals);




   drawBestFitPlane = false;

  drawPoints = computeBestFitLine(initialCloud->points);

    drawBestFitLine = false;

   computeBestFitSphere(initialCloud->points , spherecenter, sphereRadius);
    drawBestFitSphere = true;


   //   drawPoints = computeBestFitLine(initialCloud->points);
   // initialCloud->thinning(0.004);
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
    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(display);

    glutMouseFunc(mouse);
    glutMotionFunc(mouseDrag);
    glutSpecialFunc(keys);

    glewInit();

    //shaders
    create_shaders();

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslated(-initialCloud->center.x, -initialCloud->center.y, -initialCloud->center.z - initialCloud->sceneRadius);
    glutPostRedisplay();






    glutMainLoop();
    return 0;
}
