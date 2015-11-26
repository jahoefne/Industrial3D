//
// Created by moe on 05/11/15.
//

#ifndef INDUSTRIAL3D_POINT3D_H
#define INDUSTRIAL3D_POINT3D_H

#include <cmath>

class Point3D {
public:
    double x = 0.0, y = 0.0, z = 0.0;
    float r = 1;
    float g = 1;
    float b = 1;
    float size = 1;

    /** Constructors */
    Point3D(double x, double y, double z) : x(x), y(y), z(z) { }

    Point3D() : x(0), y(0), z(0) { }

    /** Move this point by another point */
    void translate(Point3D *dst) {
        x += dst->x;
        y += dst->y;
        z += dst->z;
    }

    /** A vector describing the difference between this and *dst */
    Point3D *diffVector(Point3D *dst) {
        return new Point3D(x - dst->x, y - dst->y, z - dst->z);
    }

    void rgbSize(float r, float g, float b, float size) {
        this->r = r;
        this->g = g;
        this->b = b;
        this->size = size;
    }

    /** Change Color for this point */
    void highlight() {
        r = 1;
        b = 0;
        g = 0;
    }

    /** Change Color for this point */
    void green() {
        r = 255;
        b = 25;
        g = 25;
    }

    double distanceTo(Point3D pt) {
        double d = std::sqrt(sqr(this->x - pt.x) + sqr(this->y - pt.y) + sqr(this->z - pt.z));
        return d;
    }

    double sqDistance3d(Point3D *pt) {
        const double d = sqr(x - pt->x) + sqr(y - pt->y) + sqr(z - pt->z);
        return d;
    }

    double distance3d(Point3D *pt) {
        const double d = std::sqrt(sqDistance3d(pt));
        return d;
    }


private:
    double sqr(const double &value) {
        return value * value;
    }
};

#endif //INDUSTRIAL3D_POINT3D_H
