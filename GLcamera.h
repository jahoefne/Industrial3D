#ifndef MY_GLCAMERA_H
#define MY_GLCAMERA_H

#include "Point3D.h"

class GLcamera
{
  public:
    GLcamera();

    void initializeCamera(Point3D rotationCenter, double sceneRadius);
    void setWindowSize(int winWidth, int winHeight){m_winWidth=winWidth; m_winHeight=winHeight;}
    void updateProjection();
    void zoom(double factor){m_zoomFactor*=factor; updateProjection();}
    void rotate(int x1, int y1, int x2, int y2);

    void usePerspectiveProjection(bool enable){ m_usePerspectiveProjection = enable; updateProjection();}
    bool usesPerspectiveProjection()const {return m_usePerspectiveProjection;}

  private:
    Point3D m_rotationCenter;
    double  m_sceneRadius;
    double  m_zoomFactor;
    int     m_winWidth, m_winHeight;
    double  m_zNear, m_zFar;
    bool    m_usePerspectiveProjection;
};

#endif
