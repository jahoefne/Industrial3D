//
// Created by moe on 05/11/15.
//

#ifndef INDUSTRIAL3D_POINT3D_H
#define INDUSTRIAL3D_POINT3D_H

#include <cmath>

inline double square(double value) {
    return value * value;
}

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

    //custom operators that enable vector algebra
    //these operator are marked CONST because they don't change member variables of this class
    Point3D operator + (const Point3D& p2) const; // + operation of two points (pt3 = pt1 + pt2)
    Point3D operator - (const Point3D& p2) const; // - operation of two points (pt3 = pt1 - pt2)
    Point3D operator * (double scalar) const;     // multiplication with a scalar value(pt2 = pt1 * 0.5)

    //assignments with operator
    //the operators can not be marked CONST because they do change the input
    Point3D& operator += (const Point3D& p2); // += operation of two points (pt1+= pt2  -> pt1 = pt1 + pt2)
    Point3D& operator -= (const Point3D& p2); // += operation of two points (pt1-= pt2  -> pt1 = pt1 - pt2)
    Point3D& operator *= (double scalar);     // *= multiplication with a scalar (pt1*= s  -> pt1 = pt1 * s)

    //double square(double value);                                   ///< returns the square of a value
    //double  vectorLength(const Point3D& v);                     ///< returns the length of a vector
    //double  dotProduct  (const Point3D& v1, const Point3D& v2); ///< returns the dot product of two 3d vectors
    //Point3D crossProduct(const Point3D& v1, const Point3D& v2); ///< returns the cross product of two 3d vectors
    //void    normalizeVector(Point3D& v);                        ///< normalizes a 3d vector
    double  sqDistance3d(const Point3D& v1, const Point3D& v2); ///< returns the squared Euclidean distance between two 3d points/vectors
    double  distance3d  (const Point3D& v1, const Point3D& v2); ///< returns the Euclidean distance between two 3d points/vectors

    /** Move this point by another point */
    void translate(Point3D *dst) {
        x += dst->x;
        y += dst->y;
        z += dst->z;
    }

    /** A vector describing the difference between this and *dst */
    Point3D* diffVector(Point3D *dst) {
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
        double d = std::sqrt(square(this->x - pt.x) + square(this->y - pt.y) + square(this->z - pt.z));
        return d;
    }

    double sqDistance3d(Point3D *pt) {
        const double d = square(x - pt->x) + square(y - pt->y) + square(z - pt->z);
        return d;
    }

    double distance3d(Point3D *pt) {
        const double d = std::sqrt(sqDistance3d(pt));
        return d;
    }

};


//returns the dot product of two 3d vectors
inline double dotProduct(const Point3D& v1, const Point3D& v2)
{
  return (v1.x*v2.x) + (v1.y*v2.y) + (v1.z*v2.z);
}

//returns the cross product of two 3d vectors
inline Point3D crossProduct(const Point3D& v1, const Point3D& v2)
{
  Point3D result;
  result.x = (v1.y * v2.z) - (v1.z * v2.y);
  result.y = (v1.z * v2.x) - (v1.x * v2.z);
  result.z = (v1.x * v2.y) - (v1.y * v2.x);

  return result;
}

//normalizes a 3d vector (direction vector)
inline void normalizeVector(Point3D& v)
{
  const double length = sqrt(square(v.x) + square(v.y) + square(v.z));
  if (length > 0)
  {
    v.x /= length;
    v.y /= length;
    v.z /= length;
  }
}
#endif //INDUSTRIAL3D_POINT3D_H
