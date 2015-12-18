#ifndef INDUSTRIAL3D_POINTCLOUD_H
#define INDUSTRIAL3D_POINTCLOUD_H


#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Point3D.h"
#include "K3DTree.h"

using namespace std;


/**
 * Represents a cloud of points in 3D space
 */
class PointCloud {
public:
    /** default empty constructor */
    PointCloud(){}

    /** TODO: Who ever wrote this function - please document what it does*/
    double maxAt(std::vector<double>& vector_name);

    /**
     * Loads the points of this cloud from an xyz file
     * @param fileName the xyz file which contains the coordinated of the points in an space separated ascii file
     * @r the initial red value of the points
     * @g the initial green value of the points
     * @b the initial blue value of the points
     *
     * @return != 0 if an error occured. 0 if successful
     */
    int loadPointsFromFile(std::string fileName, float r = 1, float g = 1, float b =1);


    std::vector<Point3D> points; /**< a vector representation of the points in this cloud */
    K3DTree* kdTree; /**< a vector representation of the points in this cloud */
    Point3D center;  /**< the center of the cloud */
    Point3D boundingBoxMin; /**< the minimum point of the bounding box */
    Point3D boundingBoxMax; /**< the maximum point of the bounding box */
    double sceneRadius; /**< the radius of the scene */

    /** Translate the whole cloud by a given vector
     * @param point the vector by which the cloud should be translated
     */
    void translate(Point3D* point);

    /** TODO: please document */
    Point3D* getcolor(Point3D* green, Point3D* yellow, Point3D* red,Point3D* final,float interpolation_factor);

    /** Tries to align this cloud to another cloud */
    void alignTo(PointCloud* cloud);

    /**
     * Smoothes the point cloud by a given radius
     * @param radius the smoothing radius
     * @return the smoothed point cloud
     */
    PointCloud* smooth(double radius);

    void thinning(double radius);

    void computeBBox(const std::vector<Point3D>& points);

/*
    double getMax_x();
    double getMax_y();
    double getMax_z();
    double getMin_x();
    double getMin_y();
    double getMin_z();*/
};


#endif //INDUSTRIAL3D_POINTCLOUD_H
