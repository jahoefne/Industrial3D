//
// Created by moe on 05/11/15.
//

#ifndef INDUSTRIAL3D_POINT3D_H
#define INDUSTRIAL3D_POINT3D_H

#include <cmath>

class Point3D {
public:
    double x = 0.0, y = 0.0, z = 0.0;
    signed char b = (signed char) 255;
    signed char g = (signed char) 255;
    signed char r = (signed char) 255;

    /** Constructors */
    Point3D(double x, double y, double z) : x(x), y(y), z(z) { }
    Point3D() : x(0), y(0), z(0) { }

    /** Move this point by another point */
    void translate(Point3D* dst) {
        x += dst->x;
        y += dst->y;
        z += dst->z;
    }

    /** Change Color for this point */
    void highlight(){
        r = (signed char) 255;
        b = (signed char) 0;
        g = (signed char) 0;
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
