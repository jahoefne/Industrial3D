#include "GLwidget.h"
//#include <QtOpenGL/qgl.h>
#include <QtOpenGL>
#include<QtGui/QMouseEvent>
#include "Point3D.h"

void GLwidget::initializeGL()
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void GLwidget::resizeGL(int w, int h)
{
  glViewport(0,0,w,h);

  m_camera.setWindowSize(w,h);
  m_camera.updateProjection(); //adjust projection to new window size
}

void GLwidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //clear buffers
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);   //clear background color
  glClearDepth(1.0f);                     //clear depth buffer

  //draws the scene background
  //drawBackground();

  //Draw pointclouds
  if (!m_points.empty())
  { /* Drawing Points with VertexArrays */
    glEnableClientState(GL_VERTEX_ARRAY);

    glPointSize(2);
    glColor3ub(255, 255, 255);
    glVertexPointer(3, GL_DOUBLE, sizeof(Point3D), &m_points[0]);
    glDrawArrays(GL_POINTS, 0, (unsigned int)m_points.size());

    if (!m_neighbors.empty())
    {
      glPointSize(3);
      glColor3ub(255, 0, 0);
      glVertexPointer(3, GL_DOUBLE, sizeof(Point3D), &m_neighbors[0]);
      glDrawArrays(GL_POINTS, 0, (unsigned int)m_neighbors.size());
    }

    glDisableClientState(GL_VERTEX_ARRAY);
  }



  //draw coordinate frame
  drawCoordinateAxes();

  //glBegin(GL_LINES);
  //  glColor3ub(255, 0, 0);
  //  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  //  glVertex3d(m_sceneCenter.x + m_sceneRadius, m_sceneCenter.y, m_sceneCenter.z);
  //  glColor3ub(0, 255, 0);
  //  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  //  glVertex3d(m_sceneCenter.x, m_sceneCenter.y + m_sceneRadius, m_sceneCenter.z);
  //  glColor3ub(0, 0, 255);
  //  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  //  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z + m_sceneRadius);
  //glEnd();

  ////draw bounding box
  //glPushMatrix();
  //glPushAttrib(GL_POLYGON_BIT);
  //  glColor3ub(255,255,255);
  //  Point3D S=m_bbmax-m_bbmin;
  //  glTranslated(m_bbmin.x, m_bbmin.y, m_bbmin.z);
  //  glScaled(S.x,S.y,S.z);
  //  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //draw wire frame instead of filled quads
  //  drawBox();
  //glPopAttrib();
  //glPopMatrix();

  ////draw scene center point as a sphere
  //glPushMatrix();
  //  glColor3ub(255, 255, 0);
  //  glTranslated(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  //  GLUquadric* quad = gluNewQuadric();
  //  gluSphere(quad, m_sceneRadius / 20, 30, 30);
  //  gluDeleteQuadric(quad);
  //glPopMatrix();

  update();
}

void GLwidget::mousePressEvent(QMouseEvent * e)  ///<
{
  if (e->buttons() == Qt::LeftButton)
    m_mouseLastPos = e->pos();
}

void GLwidget::mouseMoveEvent(QMouseEvent * e)   ///<
{
  //std::cout << e->pos().x() << "," << e->pos().y()<<std::endl;

  if (e->buttons() != Qt::LeftButton){ return; }

  //No action, if mouse position outside the window
  if ((*e).x()<0 || (*e).x() >= width() ){ return; }
  if ((*e).y()<0 || (*e).y() >= height()){ return; }

  const QPoint& mouseCurrentPos(e->pos());

  if (m_mouseLastPos == mouseCurrentPos) return;

  makeCurrent();
  m_camera.rotate(m_mouseLastPos.x(), m_mouseLastPos.y(), mouseCurrentPos.x(), mouseCurrentPos.y());

  //the current mouse position is the last mouse position for the next interaction
  m_mouseLastPos = mouseCurrentPos;

  update();
}

void GLwidget::wheelEvent(QWheelEvent * e)
{
  if (e->orientation() != Qt::Vertical) return;

  const double factor = e->delta()<0 ? 1.1 : 0.9;

  makeCurrent();
  m_camera.zoom(factor);
}

void GLwidget::updateScene()
{
  //check how many points we have read from file
  std::cout << "point vector contains: " << m_points.size() << " points" << std::endl;

  if (m_points.empty())
  {
    std::cout << "ERROR: no points to show...(press enter to exit)" << std::endl;
    getc(stdin);
  }

  //OK, we now compute the min and max coordinates for our bounding box
  Point3D minPoint = m_points.front(); //initialize min with the first point
  Point3D maxPoint = m_points.front(); //initialize max with the first point

  for (unsigned int i = 0; i < m_points.size(); ++i)
  {
    const Point3D& pt = m_points[i]; //do not copy but get a reference to the i-th point in the vector
    if (pt.x < minPoint.x) minPoint.x = pt.x;
    else if (pt.x > maxPoint.x) maxPoint.x = pt.x;

    if (pt.y < minPoint.y) minPoint.y = pt.y;
    else if (pt.y > maxPoint.y) maxPoint.y = pt.y;

    if (pt.z < minPoint.z) minPoint.z = pt.z;
    else if (pt.z > maxPoint.z) maxPoint.z = pt.z;
  }

  m_sceneCenter.x = (maxPoint.x + minPoint.x) / 2;
  m_sceneCenter.y = (maxPoint.y + minPoint.y) / 2;
  m_sceneCenter.z = (maxPoint.z + minPoint.z) / 2;

  //m_sceneRadius = distance3d(m_sceneCenter, maxPoint);
  //m_sceneRadius = Point3D::distance3d(m_sceneCenter);

  m_bbmin=minPoint;
  m_bbmax=maxPoint;

  std::cout << "\nBounding Box was computed:\n";
  std::cout << "minPoint is: " << minPoint.x << "," << minPoint.y << "," << minPoint.z << std::endl;
  std::cout << "maxPoint is: " << maxPoint.x << "," << maxPoint.y << "," << maxPoint.z << std::endl;

  makeCurrent();
  m_camera.initializeCamera(m_sceneCenter, m_sceneRadius); //set rotation center and scene radius to initially setup the camera
  m_camera.updateProjection(); //updateProjection to make the new point cloud fit to the screen
}

/** draws a unit box.
origin is at (0,0,0) and maximum at (1,1,1).
use glTranslate to move the origin to the minimum coordinates
afterwards use glScale(sx,sy,sz) resize the box.
*/
void GLwidget::drawBox()
{
  glBegin(GL_QUADS);
  glVertex3d(0, 0, 0); glVertex3d(1, 0, 0); glVertex3d(1, 1, 0); glVertex3d(0, 1, 0); //front
  glVertex3d(0, 0, 1); glVertex3d(1, 0, 1); glVertex3d(1, 1, 1); glVertex3d(0, 1, 1); //back
  glVertex3d(0, 0, 0); glVertex3d(0, 0, 1); glVertex3d(0, 1, 1); glVertex3d(0, 1, 0); //left
  glVertex3d(1, 0, 0); glVertex3d(1, 0, 1); glVertex3d(1, 1, 1); glVertex3d(1, 1, 0); //right
  glVertex3d(0, 0, 0); glVertex3d(1, 0, 0); glVertex3d(1, 0, 1); glVertex3d(0, 0, 1); //bottom
  glVertex3d(0, 1, 0); glVertex3d(1, 1, 0); glVertex3d(1, 1, 1); glVertex3d(0, 1, 1); //top
  glEnd();
}

//draws a unit circle with radius 1 at the origin(0,0,0)
//use glTranslate, glScale and glRotate to resize and reposition the circle
void GLwidget::drawCircle()
{
  const int segments = 180;

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < segments; ++i)
  {
    const double theta = 2.0 * 3.1415926 * double(i) / double(segments);
    glVertex2d(cos(theta), sin(theta));
  }
  glEnd();
}

//draws the coordinate system
void GLwidget::drawCoordinateAxes()
{
  //draw coordinate frame
  glBegin(GL_LINES);
  //draw line for X-Axis
  glColor3ub(255, 0, 0);
  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  glVertex3d(m_sceneCenter.x + m_sceneRadius, m_sceneCenter.y, m_sceneCenter.z);
  //draw line for Y-Axis
  glColor3ub(0, 255, 0);
  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  glVertex3d(m_sceneCenter.x, m_sceneCenter.y + m_sceneRadius, m_sceneCenter.z);
  //draw line for Z-Axis
  glColor3ub(0, 0, 255);
  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  glVertex3d(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z + m_sceneRadius);
  glEnd();

  //draw center point as a sphere
  glPushMatrix();
  glColor3ub(255, 255, 0);
  glTranslated(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  //GLUquadric* quad = gluNewQuadric();
  //gluSphere(quad, m_sceneRadius / 20, 30, 30);
  //gluDeleteQuadric(quad);
  glPopMatrix();

  glPushMatrix();
  glColor3ub(64, 164, 164);
  glTranslated(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  glScaled(m_sceneRadius, m_sceneRadius, m_sceneRadius);
  drawCircle();
  //draw another circle 90 degree rotated
  glRotated(90, 1, 0, 0);
  drawCircle();
  glPopMatrix();

  //draw a disk to indicate the scene radius
  /*glPushMatrix();
  glColor3ub(255, 0, 255);
  glTranslated(m_sceneCenter.x, m_sceneCenter.y, m_sceneCenter.z);
  GLUquadric* quad2 = gluNewQuadric();
  gluDisk(quad2, m_sceneRadius, m_sceneRadius*1.001, 100, 100);
  gluDeleteQuadric(quad2);*/

  //draw bounding box
  glPushMatrix();
  glPushAttrib(GL_POLYGON_BIT);
    glColor3ub(255,255,255);
    Point3D S=m_bbmax-m_bbmin;
    glTranslated(m_bbmin.x, m_bbmin.y, m_bbmin.z);
    glScaled(S.x,S.y,S.z);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //draw wire frame instead of filled quads
    drawBox();
  glPopAttrib();
  glPopMatrix();
}

//------------------------------------------------------------------------------
/** draws a static permanent 2d color gradient.
    Drawing static background means that we draw a 2D rectangle, which is not
    influenced by scene rotation and by the camera projection.
    Basically we just want to draw a 2D texture/image.
*/
//------------------------------------------------------------------------------
/*void GLwidget::drawBackground()
{
  makeCurrent(); //each time when we call a glXXX-Funktion QT needs the current context

  const float winWidth((float)width());
  const float winHeight((float)height());

  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  glMatrixMode(GL_PROJECTION);           //At first we select the Projection matrix
  glPushMatrix();                        //save Projektion matrix to restore it at the end
  glLoadIdentity();                      //initialize projection matrix
  gluOrtho2D(0, winWidth, 0, winHeight); //select orthographic projecton

  glMatrixMode(GL_MODELVIEW);            //now change to Modelview because we want to draw something
  glPushMatrix();
  glLoadIdentity();

  glBegin(GL_QUADS);
  glColor3ub(100,100,100); //color bottom
  glVertex2f(0.0f, 0.0f);  glVertex2f(winWidth, 0.0f);
  glColor3ub(38,38,38);  //color top
  glVertex2f(winWidth, winHeight);  glVertex2f(0.0f, winHeight);
  glEnd();

  glMatrixMode(GL_PROJECTION); //select to Projektionmatrix
  glPopMatrix();  //reset 2D-projection

  glMatrixMode(GL_MODELVIEW); //select MODELVIEW
  glPopMatrix();  //reset ModelviewMatrix to last state

  glPopAttrib();  //restore last attributes
}
*/
