#ifndef POLARDECOMPOSITION_H
#define POLARDECOMPOSITION_H

#include "btMatrix3x3.h"

class btPolarDecomposition
{
  public:
    static const btScalar DEFAULT_TOLERANCE;
    static const unsigned int DEFAULT_MAX_ITERATIONS;

    btPolarDecomposition(btScalar tolerance = DEFAULT_TOLERANCE, 
    unsigned int maxIterations = DEFAULT_MAX_ITERATIONS);

    unsigned int decompose(const btMatrix3x3& a, btMatrix3x3& u, btMatrix3x3& h) const; 
    unsigned int maxIterations() const;

  private:
    btScalar m_tolerance;
    unsigned int m_maxIterations;
};

unsigned int polarDecompose(const btMatrix3x3& a, btMatrix3x3& u, btMatrix3x3& h); 

#endif // POLARDECOMPOSITION_H

