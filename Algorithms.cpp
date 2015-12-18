#include <iostream>
#include "Algorithms.h"
#include "SVD.h"
#include <stddef.h>
#include <GL/gl.h>
#include "PointCloud.h"

/** @brief Computes and returns the center of the point cloud.
@param points vector of points
*/
Point3D computeCenter(const std::vector<Point3D> &points) {
    //compute the mean value (center) of the points cloud
    Point3D mean(0, 0, 0);

    const size_t N = points.size();
    if (N < 1) return mean; //an empty point cloud gets (0,0,0) as the center

    for (size_t i = 0; i < N; ++i) {
        mean += points[i];
    }
    mean *= 1.0 / N;

    return mean;
}

/** @brief computes the 3x3 Varianz matrix as the base for a Principal Component Analysis.
@param points vector of points
@param M      3x3 matrix
*/
void computeCovarianceMatrix3x3(const std::vector<Point3D> &points, Matrix &M) {
    M.resize(3, 3);
    const ptrdiff_t N(points.size());
    if (N < 1) return;

    //compute the mean value (center) of the points cloud
    Point3D mean = computeCenter(points);

    //Compute the entries of the (symmetric) covariance matrix
    double Mxx(0), Mxy(0), Mxz(0), Myy(0), Myz(0), Mzz(0);
#pragma omp parallel for reduction(+: Mxx,Mxy,Mxz,Myy,Myz,Mzz) //omp reduction enables parallel sum up of values
    for (ptrdiff_t i = 0; i < N; ++i) {
        const Point3D &pt = points[i];

        //generate mean-free coorinates
        const double x1(pt.x - mean.x);
        const double y1(pt.y - mean.y);
        const double z1(pt.z - mean.z);

        //Sum up the entries for the covariance matrix
        Mxx += x1 * x1;
        Mxy += x1 * y1;
        Mxz += x1 * z1;
        Myy += y1 * y1;
        Myz += y1 * z1;
        Mzz += z1 * z1;

    }

    //setting the sums to the matrix (division by N just for numerical reason if we have very large sums)
    M(0, 0) = Mxx / N;
    M(0, 1) = Mxy / N;
    M(0, 2) = Mxz / N;
    M(1, 0) = M(0, 1);
    M(1, 1) = Myy / N;
    M(1, 2) = Myz / N;
    M(2, 0) = M(0, 2);
    M(2, 1) = M(1, 2);
    M(2, 2) = Mzz / N;

    for(int i=0; i<9; ++i)
        printf("%lf\n",M[i] );

}

/** @brief computes best-fit approximations.
@param points vector of points
*/
vector<Point3D>* computeBestFits(const std::vector<Point3D> &points) {
    Matrix M(3, 3);

    const Point3D center = computeCenter(points);
    computeCovarianceMatrix3x3(points, M);
    SVD::computeSymmetricEigenvectors(M);

    const Point3D ev0(M(0, 0), M(1, 0), M(2,
                                          0)); //first column of M == Eigenvector corresponding to the largest Eigenvalue == direction of biggest variance
   // const Point3D ev1(M(0, 1), M(1, 1), M(2, 1));
    const Point3D ev2(M(0, 2), M(1, 2), M(2,
                                          2)); //third column of M == Eigenvector corresponding to the smallest Eigenvalue == direction of lowest variance

    //best-fit line
    std::cout << "*** Best-fit line ***\n";
    std::cout << "Point    : " << center.x << ", " << center.y << ", " << center.z << std::endl;
    std::cout << "Direction: " << ev0.x << ", " << ev0.y << ", " << ev0.z << std::endl;

    //computing the mean distance to line
    double meanDistance = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        meanDistance += distancePt2Line(points[i], center, ev0);
    }
    meanDistance /= points.size();
    std::cout << "mean distance to line: " << meanDistance << std::endl;


    std::cout << "\n";
    //best-fit plane
    std::cout << "*** Best-fit plane ***\n";
    std::cout << "Point    : " << center.x << ", " << center.y << ", " << center.z << std::endl;
    std::cout << "Direction: " << ev2.x << ", " << ev2.y << ", " << ev2.z << std::endl;

    //computing the mean distance to line
    meanDistance = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        meanDistance += std::abs(distancePt2Plane(points[i], center, ev2));
    }
    meanDistance /= points.size();
    std::cout << "mean distance to plane: " << meanDistance << std::endl;


    double param = 0.1;
    double param2 = 1.0;

    vector<Point3D>* drawPoints = new vector<Point3D>();
    drawPoints->push_back(center + ev0*param);
    drawPoints->push_back(center - ev0*param);
    drawPoints->push_back(center + ev2*param);
    drawPoints->push_back(center - ev2*param);
    return drawPoints;

}


double sqr(double value)
{
    return value*value;
}

double  vectorLength(const Point3D& v)
{
    double length = sqrt(sqr(v.x) + sqr(v.y) + sqr(v.z));
    return length;
}

/** @ingroup grpMath
    @brief returns the dot product of two 3d vectors.
    @param v1,v2 two points
    @return dot product
*/
double dotProduct(const Point3D& v1, const Point3D& v2)
{
    return (v1.x*v2.x) + (v1.y*v2.y) + (v1.z*v2.z);
}

/** @ingroup grpMath
    @brief returns the cross product of two 3d vectors.
    @param v1,v2 two points
    @return vector
*/
Point3D crossProduct(const Point3D& v1, const Point3D& v2)
{
    Point3D result;
    result.x = (v1.y * v2.z) - (v1.z * v2.y);
    result.y = (v1.z * v2.x) - (v1.x * v2.z);
    result.z = (v1.x * v2.y) - (v1.y * v2.x);

    return result;
}

/** @brief Computes the distance of a point to a 3D line.
    @param point         point
    @param pointOnLine   a point on the line (e.g. "center" point)
    @param lineDirection vector respresenting the 3d direction of the line (must be a unit vector -> length==1)
*/
double distancePt2Line(const Point3D &point, const Point3D &pointOnLine, const Point3D &lineDirection) {
    const Point3D PQ = pointOnLine - point;
    double distance = vectorLength(crossProduct(PQ, lineDirection));

    return distance;
}

/** @brief Computes the distance of a point to a plane.
@param point          point
@param pointOnPlane   a point on the plane (e.g. "center" point)
@param planeDirection vector respresenting the 3d direction of the plane (must be a unit vector -> length==1)
*/
double distancePt2Plane(const Point3D &point, const Point3D &pointOnPlane, const Point3D &planeDirection) {
    const Point3D PQ = pointOnPlane - point;
    double distance = dotProduct(PQ, planeDirection);

    return distance;
}
