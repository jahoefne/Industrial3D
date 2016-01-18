#ifndef MY_ALGORITHMS_H
#define MY_ALGORITHMS_H

#include <vector>
#include "Point3D.h"
#include "Matrix.h"


Point3D computeCenter(const std::vector<Point3D>& points);                  ///< Computes and returns the center of the point cloud
void computeCoarianceMatrix(const std::vector<Point3D>& points, Matrix& M); ///< Coputes the 3x3 covariance matrix
std::vector<Point3D> *computeBestFits(const std::vector<Point3D>& points);                   ///< Computes best-fit line and plane


std::vector<Point3D> *computeBestFitLine (const std::vector<Point3D>& points);  ///< Computes best-fit line
//void computeBestFitPlane(const std::vector<Point3D>& points, std::vector<Point3D>& corners);  ///< Computes best-fit plane
std::vector<Point3D> *computeBestFitPlane(const std::vector<Point3D>& points);  ///< Computes best-fit plane
void computeBestFitSphere(const std::vector<Point3D>& points , Point3D& center, double& radius); //, Point3d& center, double& radius);

double distancePt2Line (const Point3D& point, const Point3D& pointOnLine , const Point3D& lineDirection );  ///< distance point-to-line (3d)
double distancePt2Plane(const Point3D& point, const Point3D& pointOnPlane, const Point3D& planeDirection);  ///< distance point-to-plane

#endif //MY_ALGORITHMS_H
