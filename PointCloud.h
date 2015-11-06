//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#ifndef INDUSTRIAL3D_POINTCLOUD_H
#define INDUSTRIAL3D_POINTCLOUD_H


#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Point3D.h"

using namespace std;


class PointCloud {
public:
    int loadPointsFromFile(string fileName);
    std::vector<Point3D*> points;
    Point3D center;
    Point3D boundingBoxMin;
    Point3D boundingBoxMax;
    double sceneRadius;
    void print();
};


#endif //INDUSTRIAL3D_POINTCLOUD_H
