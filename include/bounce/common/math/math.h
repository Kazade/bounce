/*
* Copyright (c) 2016-2016 Irlan Robson http://www.irlan.net
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_MATH_H
#define B3_MATH_H

#include <cmath>
#include <cstdlib> // For abs() with integral types
#include <bounce/common/settings.h>

inline bool b3IsInf(float32 x)
{
	return std::isinf(x);
}

inline bool b3IsNaN(float32 x)
{
	return std::isnan(x);
}

inline bool b3IsValid(float32 fx)
{
	i32 ix = *(i32*)(&fx);
	return (ix & 0x7F800000) != 0x7F800000;
}

inline float32 b3Sqrt(float32 x) 
{
	return std::sqrt(x);
}

template <class T>
inline T b3Abs(T x) 
{
	return std::abs(x);
}

template <class T>
inline T b3Min(T a, T b) 
{
	return a < b ? a : b;
}

template <class T>
inline T b3Max(T a, T b) 
{
	return a > b ? a : b;
}

template <class T>
inline T b3Clamp(T a, T low, T high) 
{
	return b3Max(low, b3Min(a, high));
}

template <class T>
inline void b3Swap(T& a, T& b) 
{
	T tmp = a;
	a = b;
	b = tmp;
}

template <class T>
inline T b3Sign(T x)
{
	return T(T(0) < x) - T(T(x) < T(0));
}

template <class T>
inline u32 b3UniqueCount(T* V, u32 N)
{
	u32 count = 0;
	for (u32 i = 0; i < N; ++i) 
	{
		u32 j;
		for (j = 0; j < N; ++j)
		{
			if (V[i] == V[j])
			{
				break;
			}
		}		
		if (i == j)
		{
			++count;
		}
	}
	return count;
}

#endif
