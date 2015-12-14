#include <cmath>        //for standard C/C++ math functions
#include "Point3D.h"

//custom operator that enables the + operation of two points (pt3 = pt1 + pt2)
Point3D Point3D::operator + (const Point3D& p2) const
{
  Point3D result;
  result.x = x + p2.x;
  result.y = y + p2.y;
  result.z = z + p2.z;
  return result;
}

//custom operator that enables the - operation of two points (pt3 = pt1 - pt2)
Point3D Point3D::operator - (const Point3D& p2) const
{
  Point3D result;
  result.x = x - p2.x;
  result.y = y - p2.y;
  result.z = z - p2.z;
  return result;
}
//custom operator that enables the multiplication with a scalar value (pt2 = pt1 * 0.5)
Point3D Point3D::operator * (double scalar) const
{
  Point3D result;
  result.x = scalar * x;
  result.y = scalar * y;
  result.z = scalar * z;
  return result;
}

//custom operator that enables the += operation (pt1 += pt2 -> pt1 = pt1 + pt2)
Point3D& Point3D::operator += (const Point3D& p2)
{
  x += p2.x;
  y += p2.y;
  z += p2.z;
  return *this;
}

//custom operator that enables the -= operation (pt1 -= pt2 -> pt1 = pt1 - pt2)
Point3D& Point3D::operator -= (const Point3D& p2)
{
  x -= p2.x;
  y -= p2.y;
  z -= p2.z;
  return *this;
}

//custom operator that enables the += operation (pt1 *= 2 -> pt1 = pt1 * s)
Point3D& Point3D::operator *= (double scalar)
{
  x *= scalar;
  y *= scalar;
  z *= scalar;
  return *this;
}

//returns the square of a value (unfortunately C++ does not provide this function itself...)
/*double sqr(double value)
{
  return value*value;
}
*/
//returns the length of a vector
double  vectorLength(const Point3D& v)
{
  double length = sqrt(square(v.x) + square(v.y) + square(v.z));
  return length;
}

//dot, cross, normalize were here

///< returns the squared Euclidean distance between two 3d points/vectors
double sqDistance3d(const Point3D& v1, const Point3D& v2)
{
  double d = square(v1.x - v2.x) + square(v1.y - v2.y) + square(v1.z - v2.z);
  return d;
}

//returns the Euclidean distance between two 3d points / vectors
double distance3d(const Point3D& v1, const Point3D& v2)
{
  const double d = std::sqrt(sqDistance3d(v1,v2));
  return d;
}
