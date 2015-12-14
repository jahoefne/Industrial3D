#ifndef MY_MATRIX_H
#define MY_MATRIX_H

#include <vector>

/** @brief class that implements a MxN matrix.
    @details the ordering is row-first
*/
class Matrix
{
  public:
    Matrix():m_M(0), m_N(0){}
    Matrix(size_t m, size_t n): m_M(m), m_N(n){ m_data.resize(m*n, 0.0); }

    void resize(size_t m, size_t n) { m_data.resize(m*n, 0.0); m_M=m; m_N=n;}
    const size_t M() const {return m_M;} //number of rows
    const size_t N() const {return m_N;} //number of columns

    //Element access by (row, column)-operator
          double& operator() (size_t r, size_t c)      { return m_data[(r*m_N) + c]; }
    const double& operator() (size_t r, size_t c)const { return m_data[(r*m_N) + c]; }

    //Direct element access via []-operator
          double& operator[] (size_t index)      { return m_data[index]; }
    const double& operator[] (size_t index)const { return m_data[index]; }

  private:
    size_t m_M, m_N;            ///< dimensions of the matrix 
    std::vector<double> m_data; ///< storage for the matrix elements
};




#endif //MY_MATRIX_H