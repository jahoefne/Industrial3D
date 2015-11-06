//
// Created by moe on 05/11/15.
//

#ifndef INDUSTRIAL3D_POINT3D_H
#define INDUSTRIAL3D_POINT3D_H

#include <cmath>

class Point3D {
public:
    double x, y, z;
    Point3D(double x, double y, double z) : x(x), y(y), z(z) { }
    Point3D() : x(0), y(0), z(0) { }
    void translate(Point3D* dst) {
        x += dst->x;
        y += dst->y;
        z += dst->z;
    }

    double distanceTo(Point3D pt){
        double d = std::sqrt( sqr(this->x - pt.x) + sqr(this->y - pt.y) + sqr(this->z - pt.z) );
        return d;
    }

private:
    double sqr(const double& value){
        return value*value;
    }
};

#endif //INDUSTRIAL3D_POINT3D_H
