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

#ifndef B3_MAT_H
#define B3_MAT_H

#include <bounce/common/math/math.h>

// A vector stored in column-major order.
template<u32 n>
struct b3Vec
{
	b3Vec() { }

	const float32& operator[](u32 i) const
	{
		return e[i];
	}
	
	float32& operator[](u32 i)
	{
		return e[i];
	}
	
	void operator+=(const b3Vec<n>& v)
	{
		for (u32 i = 0; i < n; ++i)
		{
			e[i] += v[i];
		}
	}

	float32 e[n];
};

template<u32 n>
inline b3Vec<n> operator-(const b3Vec<n>& v)
{
	b3Vec<n> result;
	for (u32 i = 0; i < n; ++i)
	{
		result[i] = -v[i];
	}
	return result;
}

// A matrix stored in column-major order.
template<u32 n, u32 m>
struct b3Mat
{
	b3Mat() { }

	const float32& operator()(u32 i, u32 j) const
	{
		return e[i + n * j];
	}

	float32& operator()(u32 i, u32 j)
	{
		return e[i + n * j];
	}

	float32 e[n * m];
};

// Solve Ax = b.
// It doesn't compute the inverse. 
// Therefore, is more efficient.
// Returns false if the matrix is singular.
// Warning: Make sure to pass a copy of the original matrix to the function. A will be invalidated.
bool b3Solve(float32* b, float32* A, u32 n);

#endif