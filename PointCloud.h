//
// Created by Jan Moritz Hoefner on 23/10/15.
//

#ifndef INDUSTRIAL3D_POINTCLOUD_H
#define INDUSTRIAL3D_POINTCLOUD_H


#include <vector>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;


class PointCloud {
private:
    class Point3D {
    public:
        double x, y, z;
        Point3D(double x, double y, double z) : x(x), y(y), z(z) { }
    };

public:
    int loadPointsFromFile(string fileName);
    std::vector<Point3D*> points;
    void print();
    void draw();
};


#endif //INDUSTRIAL3D_POINTCLOUD_H
