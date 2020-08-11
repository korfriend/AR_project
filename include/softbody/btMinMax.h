#ifndef BT_GEN_MINMAX_H
#define BT_GEN_MINMAX_H

#include "btScalar.h"

template <class T>
SIMD_FORCE_INLINE const T& btMin(const T& a, const T& b) 
{
  return a < b ? a : b ;
}

template <class T>
SIMD_FORCE_INLINE const T& btMax(const T& a, const T& b) 
{
  return  a > b ? a : b;
}

template <class T>
SIMD_FORCE_INLINE const T& btClamped(const T& a, const T& lb, const T& ub) 
{
	return a < lb ? lb : (ub < a ? ub : a); 
}

template <class T>
SIMD_FORCE_INLINE void btSetMin(T& a, const T& b) 
{
    if (b < a) 
	{
		a = b;
	}
}

template <class T>
SIMD_FORCE_INLINE void btSetMax(T& a, const T& b) 
{
    if (a < b) 
	{
		a = b;
	}
}

template <class T>
SIMD_FORCE_INLINE void btClamp(T& a, const T& lb, const T& ub) 
{
	if (a < lb) 
	{
		a = lb; 
	}
	else if (ub < a) 
	{
		a = ub;
	}
}

#endif //BT_GEN_MINMAX_H
