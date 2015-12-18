
#include "SVD.h"
#include <assert.h>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <stddef.h>

namespace SVD
{

//------------------------------------------------------------------------------
/** @brief Computes the eigenvectors of symmetric square matrix with a SVD.
    @details  The singular values of a symmetric square matrix are the eigenvalues.
              Also, in this case the orthogonale matrices U and V are identical and 
              contain the eigenvector in the columns.
    @param U[in,out] MxM-Matrix from which the eigenvectors are derived. It is replaced by 
                     a MxM-matrix that contains the eigenvector in its columns (sorted in decending order of the singular values).
*/
//------------------------------------------------------------------------------
void computeSymmetricEigenvectors(Matrix& U)
{ 
  Matrix V;
  std::vector<double> S;
  decomposeMatrix(U,S,V);  //compute SVD
}

/** @brief computes the singular value decomposition of a matrix.
    @details decomposes a MxN-matrix (M >= N). 
    
    SVD-Methods  base on the theorem, that every MxN matrix A can be decomposed and described as the 
    product of an MxN column-orthogonal matrix U, a NxN diagonal matrix S (here we use a vector to store the diagonal elements) 
    The elements in S are the singular values and they are larger or equal 0) as well as a transposed orthogonal NxN-matrix V.
    The function re-sorts the singular values in decending order (and re-orders the corresponding matrix columns).
    The algorithmus is from LAPACK, but the source is derived from JAMA (public domain).
    
    @param U  Matrix to decompose (Its contents are changed, dimension stays MxN).
    @param S  Vector of singular values
    @param V  orthogonal NxN-Matrix
*/
//------------------------------------------------------------------------------
void decomposeMatrix(Matrix& U, std::vector<double>& S, Matrix& V)
{
  const size_t m = U.M();
  const size_t n = U.N();

  V.resize(U.N(), U.N());
printf("%d %d\n",V.M(), V.N());
  assert(m>=n && m>0 && n>0);
  
  S.resize(n);
  
  std::vector<double> e    (n);
  std::vector<double> work (m);
  
  // Reduction to bidiagonal matrix. The diagonal elements are stored in S.
  // The superdiagonal elements (right-handed side over the diagonal) are stored in e.
  const size_t nct = std::min(m-1,n);
  const size_t nrt = std::max(size_t(0),std::min(n-2,m));
  const size_t K   = std::max(nct,nrt);
  
  printf("%d %d %d\n", nct, nrt,K);

  for (size_t k = 0; k < K; ++k)
  {
    if (k < nct)
    {
      // computes transformation for k-th column, the k-th diagonal  then is in S[k].
      S[k] = 0;
      for (size_t i = k; i < m; ++i) {
        S[k] = hypotenuse(S[k],U(i,k));
      }
      
      if (S[k] != 0.0)
      {
        if (U(k,k) < 0.0){
          S[k] = -S[k];
        }
        
        const double invSk=1.0/S[k];
        for (size_t i = k; i < m; ++i){
          U(i,k) *= invSk;
        }
        U(k,k) += 1.0;
      }
      S[k] = -S[k];
    }
    for (size_t j = k+1; j < n; ++j)
    {
      if ((k < nct) & (S[k] != 0.0))
      {
        //apply transformation
        double t = 0;
        for (size_t i = k; i < m; ++i){
          t += U(i,k)*U(i,j);
        }
        t = -t/U(k,k);
        for (size_t i = k; i < m; ++i) {
          U(i,j) += t*U(i,k);
        }
      }

      //store k-th row of U in e (for following row-transformation).
      e[j] = U(k,j);
    }
    if (k < nrt)
    {
      // computes transformation for k-th row (the k-th Super-diagonal is in e[k].
      e[k] = 0;
      for (size_t i = k+1; i < n; ++i) {
        e[k] = hypotenuse(e[k],e[i]);
      }
      if (e[k] != 0.0)
      {
        if (e[k+1] < 0.0) {
          e[k] = -e[k];
        }
        for (size_t i = k+1; i < n; ++i) {
          e[i] /= e[k];
        }
        e[k+1] += 1.0;
      }
      e[k] = -e[k];
      
      if ((k+1 < m) & (e[k] != 0.0))
      {
        // apply transformation.
        for (size_t i = k+1; i < m; ++i) {
          work[i] = 0.0;
          for (size_t j = k+1; j < n; ++j) {
            work[i] += e[j]*U(i,j);
          }
        }
        for (size_t j = k+1; j < n; ++j) {
          const double t = -e[j]/e[k+1];
          for (size_t i = k+1; i < m; ++i) {
            U(i,j) += t*work[i];
          }
        }
      }
      
      // store transfomation in V for the following backwards-multiplication.
      for (size_t i = k+1; i < n; ++i) {
        V(i,k) = e[i];
      }
      
    }
  }

  //////////////////////////////////////////////////
  //// Build final Bidiagonal-Matrix of Order p ////
  //////////////////////////////////////////////////

  ptrdiff_t p = std::min(n,m+1);
  if (nct   < n)        { S[nct] = U(nct,nct);}
  if (m     < size_t(p)){ S[p-1] = 0.0;       }
  if (nrt+1 < size_t(p)){ e[nrt] = U(nrt,p-1);}
  e[p-1] = 0.0;

  /////////////////////////////
  ///// Build / Complete U ////
  /////////////////////////////
  for (size_t j = nct; j < n; ++j) {
    for (size_t i = 0; i < m; ++i) {
      U(i,j) = 0.0;
    }
    U(j,j) = 1.0;
  }
  
  for (ptrdiff_t k_ = nct-1; k_ >= 0; --k_)
  {
    const size_t k(k_);
    if (S[k] != 0.0)
    {
      for (size_t j = k+1; j < n; ++j) {
        double t = 0;
        for (size_t i = k; i < m; ++i) {
          t += U(i,k)*U(i,j);
        }
        t = -t/U(k,k);
        for (size_t i = k; i < m; ++i) {
          U(i,j) += t*U(i,k);
        }
      }
      for (size_t i = k; i < m; ++i ) {
        U(i,k) = -U(i,k);
      }
      U(k,k) = 1.0 + U(k,k);
      for (size_t i = 0; i < k; ++i) {
        U(i,k) = 0.0;
      }
    } 
    else {
      for (size_t i = 0; i < m; ++i) {
        U(i,k) = 0.0;
      }
      U(k,k) = 1.0;
    }
  }//for
  //////////////////////


  /////////////////////////////
  ///// Build / complete V ////
  /////////////////////////////
  for (ptrdiff_t kk = n-1; kk >= 0; --kk)
  {
    const size_t k(kk);
    if ( (k < nrt) & (e[k] != 0.0) ) {
      for (size_t j = k+1; j < n; ++j){
        double t = 0;
        for (size_t i = k+1; i < n; ++i) {
          t += V(i,k)*V(i,j);
        }
        t = -t/V(k+1,k);
        for (size_t i = k+1; i < n; ++i) {
          V(i,j) += t*V(i,k);
        }
      }
    }
    for (size_t i = 0; i < n; ++i) {
      V(i,k) = 0.0;
    }
    V(k,k) = 1.0;
  }
  //////////////////////

  //Main Iteration-Loop for singular values.
  const ptrdiff_t pp   = p-1;
  size_t          iter = 0;
  const double    eps  = std::numeric_limits<double>::epsilon();
  const double    tiny = std::numeric_limits<double>::min();
  
  while (p > 0)
  {
    ptrdiff_t k;    //counter
    size_t    task; //task depending on condition of S and e

    if(iter>100) break;

    //Analyze S und e vor neglectable elements and set appropriate task

    // task = 1     s(p) and e[k-1] are neglectable and k<p
    // task = 2     s(k) is neglectable and k<p
    // task = 3     e[k-1] is neglectable, k<p, but s(k), ..., s(p) are not neglectable (apply QR-Shift).
    // task = 4     e(p-1) is neglectable, convergence reached.

    for (k = p-2; k >= -1; --k)
    {
      if (k == -1) {
        break;
      }
      if (fabs(e[k]) <= tiny + eps*(fabs(S[k]) + fabs(S[k+1])))
      {
        e[k] = 0.0;
        break;
      }
    }
    if (k == p-2) {
      task = 4;
    }
    else
    {
      ptrdiff_t ks;
      for (ks = p-1; ks >= k; --ks)
      {
        if (ks == k) {
          break;
        }
        const double t = (ks != p   ? fabs(e[ks])  : 0.) + 
                         (ks != k+1 ? fabs(e[ks-1]): 0.);
        if (fabs(S[ks]) <= (tiny + eps*t) ){
          S[ks] = 0.0;
          break;
        }
      }
      if      (ks == k)   { task = 3; } 
      else if (ks == p-1) { task = 1; } 
      else                { task = 2; k = ks; }
    }
    ++k;

    switch (task) //execute tasks (see above).
    {
      case 1: //Reduction of neglectable S(p).
      {
        double f= e[p-2];
        e[p-2]  = 0.0;
        for (ptrdiff_t j = p-2; j >= k; --j)
        {
          double t        = hypotenuse(S[j],f);
          const double cs = S[j]/t;
          const double sn = f/t;
          S[j] = t;
          if (j != k){
            f     = -sn*e[j-1];
            e[j-1]=  cs*e[j-1];
          }

          for (size_t i = 0; i < n; ++i){
            t       =  cs*V(i,j) + sn*V(i,p-1);
            V(i,p-1)= -sn*V(i,j) + cs*V(i,p-1);
            V(i,j)  = t;
          }

        }
      }
      break;

      case 2: //Divide if neglectable S(k)
      {
        double f = e[k-1];
        e[k-1] = 0.0;
        for (ptrdiff_t j = k; j < p; ++j)
        {
          double t        = hypotenuse(S[j],f);
          const double cs = S[j]/t;
          const double sn = f/t;
          S[j]= t;
          f   = -sn*e[j];
          e[j]= cs*e[j];
          for (size_t i = 0; i < m; ++i) {
            t       =  cs*U(i,j) + sn*U(i,k-1);
            U(i,k-1)= -sn*U(i,j) + cs*U(i,k-1);
            U(i,j)  = t;
          }
        }
      }
      break;

      case 3: //Apply QR-Step / Shift.
      {
        const double scale = std::max(std::max(std::max(std::max(
                             fabs(S[p-1]),fabs(S[p-2])),fabs(e[p-2])), 
                             fabs(S[k])),fabs(e[k]));
        const double sp    = S[p-1]/scale;
        const double spm1  = S[p-2]/scale;
        const double epm1  = e[p-2]/scale;
        const double sk    = S[k]/scale;
        const double ek    = e[k]/scale;
        const double b     = ((spm1 + sp)*(spm1 - sp) + epm1*epm1)/2.0;
        const double c     = (sp*epm1)*(sp*epm1);
        double       shift = 0.0;
        if ((b != 0.0) | (c != 0.0))
        {
          shift = std::sqrt(b*b + c);
          if (b < 0.0) {
            shift = -shift;
          }
          shift = c/(b + shift);
        }
        double f = (sk + sp)*(sk - sp) + shift;
        double g = sk*ek;

        for (ptrdiff_t j = k; j < p-1; ++j)
        {
          double t  = hypotenuse(f,g);
          double cs = f/t;
          double sn = g/t;
          if (j != k) {
            e[j-1] = t;
          }
          f     = cs*S[j] + sn*e[j];
          e[j]  = cs*e[j] - sn*S[j];
          g     = sn*S[j+1];
          S[j+1]= cs*S[j+1];
          
          for (size_t i = 0; i < n; ++i){
            t       =  cs*V(i,j) + sn*V(i,j+1);
            V(i,j+1)= -sn*V(i,j) + cs*V(i,j+1);
            V(i,j)  = t;
          }
          
          t     = hypotenuse(f,g);
          cs    = f/t;
          sn    = g/t;
          S[j]  = t;
          f     =  cs*e[j] + sn*S[j+1];
          S[j+1]= -sn*e[j] + cs*S[j+1];
          g     = sn*e[j+1];
          e[j+1]= cs*e[j+1];
          if ( j < ptrdiff_t(m-1) )
          {
            for (size_t i = 0; i < m; ++i) {
              t       =  cs*U(i,j) + sn*U(i,j+1);
              U(i,j+1)= -sn*U(i,j) + cs*U(i,j+1);
              U(i,j)  = t;
            }
          }
        }
        e[p-2] = f;
        ++iter;
      }
      break;

      case 4: // Convergence
      {
        // make singular values positive (and adjust V accordingly)
        if (S[k] <= 0.0)
        {
          S[k] = (S[k] < 0.0 ? -S[k] : 0.0);
          for (ptrdiff_t i = 0; i <= pp; ++i) {
            V(i,k) = -V(i,k);
          }
        }

        // sort singular values in decending order (and re-sort U and V accordingly).
        while (k < pp)
        {
          if (S[k] >= S[k+1]) {
            break;
          }
          std::swap(S[k],S[k+1]);
          if ( k < ptrdiff_t(n-1) ) {
            for (size_t i = 0; i < n; ++i){
              std::swap(V(i,k+1),V(i,k));
            }
          }
          if ( k < ptrdiff_t(m-1) ) {
            for (size_t i = 0; i < m; ++i) {
              std::swap(U(i,k+1),U(i,k));
            }
          }
          ++k;
        }
        iter = 0;
        --p;
      }
      break;
    }
  }
}

} //namespace SVD
