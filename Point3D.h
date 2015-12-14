//
// Created by moe on 05/11/15.
//

#ifndef INDUSTRIAL3D_POINT3D_H
#define INDUSTRIAL3D_POINT3D_H

#include <cmath>


/**
 * This class defines a point in 3D space, and a corresponding color and size.
 * Obejcts of this class are used in the K3DTree and PointCloud classes.
 */
class Point3D {
public:
    double x = 0.0;
    /**< The x coordinates of the point */
    double y = 0.0;
    /**< The y coordinates of the point */
    double z = 0.0;
    /**< The z coordinates of the point */

    float r = 1;
    /**< The red value of the point */
    float g = 1;
    /**< The green value of the point */
    float b = 1;
    /**< The blue value of the point */
    float size = 1; /**< The size of the point */

    /** @brief custom operator that enables the + operation of two points.
        \code{.cpp}
        Point3d p1, p2;
        Point3d p3 = p1 + p2;
        \endcode
      @param p2 point that should be added
    */
    Point3D operator+(const Point3D &p2) const {
        Point3D result;
        result.x = x + p2.x;
        result.y = y + p2.y;
        result.z = z + p2.z;
        return result;
    }

    /** @brief custom operator that enables the - operation of two points.
        \code{.cpp}
        Point3d p1, p2;
        Point3d p3 = p1 - p2;
        \endcode
      @param p2 point that should be subtracted
    */
    Point3D operator-(const Point3D &p2) const {
        Point3D result;
        result.x = x - p2.x;
        result.y = y - p2.y;
        result.z = z - p2.z;
        return result;
    }

    /** @brief custom operator that enables the multiplication with a scalar value.
        \code{.cpp}
        Point3d p1;
        Point3d p2 = p1*0.5;
        \endcode
      @param scalar scalar value the point should be multiplied with.
    */
    Point3D operator*(double scalar) const {
        Point3D result;
        result.x = scalar * x;
        result.y = scalar * y;
        result.z = scalar * z;
        return result;
    }

    /** @brief custom operator that enables the += operation.
        \code{.cpp}
        Point3d p1,p2;
        p1 += p2; // -> p1 = p1 + p2
        \endcode
        @param p2 point that should be added to the current (left-hand side) point.
    */
    Point3D &operator+=(const Point3D &p2) {
        x += p2.x;
        y += p2.y;
        z += p2.z;
        return *this;
    }

    /** @brief custom operator that enables the -= operation.
        \code{.cpp}
        Point3d p1,p2;
        p1 -= p2; // -> p1 = p1 - p2
        \endcode
      @param p2 point that should be subtracted from the current (left-hand side) point.
    */
    Point3D &operator-=(const Point3D &p2) {
        x -= p2.x;
        y -= p2.y;
        z -= p2.z;
        return *this;
    }

    /** @brief custom operator that enables the += operation.
        \code{.cpp}
        Point3d p1,p2;
        p1 *= 2.0 -> p1 = p1 * scalar)
        \endcode
      @param scalar scalar value the current (left-hand side) point should be multiplied with.
    */
    Point3D &operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }


    /** Constructor initalize the point with x,y,z values
     */
    Point3D(double x, double y, double z) : x(x), y(y), z(z) { }

    Point3D() : x(0), y(0), z(0) { }

    /**
     * Translate this point by another point
     * @param dst the vector by which this point should be moved
     */
    void translate(Point3D *dst) {
        x += dst->x;
        y += dst->y;
        z += dst->z;
    }

    /**
     * Compute a vector describing the difference between this and dst
     * @param dst the destination point of the vector
     * @return the difference between this point and dst
     */
    Point3D *diffVector(Point3D *dst) {
        return new Point3D(x - dst->x, y - dst->y, z - dst->z);
    }

    /**
     * Setter for the r, g, b, and size values
     * @param r red color value
     * @param g green color value
     * @param b blue color value
     * @param size the size of the point
     */
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

    /*
     * Highlight the point, changes the color of the point to red
     */
    void green() {
        r = 255;
        b = 25;
        g = 25;
    }

    /**
     * Computes the distance between this point and another point
     * @param pt the distant point
     * @return the distance between this and pt
     */
    double distanceTo(Point3D pt) {
        double d = std::sqrt(sqr(this->x - pt.x) + sqr(this->y - pt.y) + sqr(this->z - pt.z));
        return d;
    }

    /**
     * Computes the squared distance between this point and another point
     * @param pt the distant point
     * @return the squarey distance between this and pt
     */
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
