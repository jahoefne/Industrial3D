#ifndef MY_SVD_H
#define MY_SVD_H

#include <limits>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include "Matrix.h"
#include <cmath>
#include <stdio.h>

/** @brief In this namespace the singular value decomposition is implemented.
*/
namespace SVD
{
  void decomposeMatrix (Matrix& U, std::vector<double>& S, Matrix& V);  ///< computes the SV decomposition
  void computeSymmetricEigenvectors(Matrix& U); ///< computes the Eigenvectors for a symmetric square Matrix with SVD

  /** @brief returns the square of a value.
      @param v  values
      @return   squared value
  */
  inline double sqr(const double v){ return v*v;} 

  /** @brief returns the hypotenuse of two values.
      @details It is just the Euklidean distance but takes care for numerical underflow and overflow.
      @param v1,v2  values
      @return       hypotenuse
  */
  inline double hypotenuse(const double& v1, const double& v2)
  {
     //return sqrt(v1*v1 + v2*v2);

    const double a(std::abs(v1)), b(std::abs(v2));
    printf("%lf %lf\n", a, b);
    return (a > b ? a*sqrt(1.0 + sqr(b / a)) :
                    (b == 0.0 ? 0.0 : b*sqrt(1.0 + sqr(a / b))));
  }
}

#endif  // MY_SVD_H
