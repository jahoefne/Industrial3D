#ifndef INDUSTRIAL3D_POINTCLOUD_H
#define INDUSTRIAL3D_POINTCLOUD_H


#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Point3D.h"
#include "K3DTree.h"

using namespace std;


class PointCloud {
public:
    PointCloud(){

    };
    int loadPointsFromFile(string fileName);
    std::vector<Point3D> points;
    K3DTree* kdTree;
    Point3D center;
    Point3D boundingBoxMin;
    Point3D boundingBoxMax;
    double sceneRadius;
    void print();
};


#endif //INDUSTRIAL3D_POINTCLOUD_H
