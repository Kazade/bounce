/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#ifndef B3_MAT_22_H
#define B3_MAT_22_H

#include <bounce/common/math/vec2.h>

// A 2-by-2 matrix stored in column-major order.
struct b3Mat22
{
	// Does nothing for performance.
	b3Mat22() { }

	// Set this matrix from two vectors.
	b3Mat22(const b3Vec2& _x, const b3Vec2& _y) : x(_x), y(_y) { }
	
	// Read an indexed column vector from this matrix.
	const b3Vec2& operator[](u32 i) const
	{
		return (&x)[i];
	}

	// Write an indexed column vector to this matrix.
	b3Vec2& operator[](u32 i)
	{
		return (&x)[i];
	}

	// Read an indexed element from this matrix.
	scalar operator()(u32 i, u32 j) const
	{
		return (&x.x)[i + 2 * j];
	}

	// Write an indexed element from this matrix.
	scalar& operator()(u32 i, u32 j)
	{
		return (&x.x)[i + 2 * j];
	}

	// Set this matrix to the zero matrix.
	void SetZero()
	{
		x.SetZero();
		y.SetZero();
	}

	// Set this matrix to the identity matrix.
	void SetIdentity()
	{
		x.Set(scalar(1), scalar(0));
		y.Set(scalar(0), scalar(1));
	}

	// Solve Ax = b. 
	// It doesn't compute the inverse. 
	// Therefore, is more efficient.
	// Returns the zero vector if the matrix is singular.
	b3Vec2 Solve(const b3Vec2& b) const;

	b3Vec2 x, y;
};

// Zero matrix
extern const b3Mat22 b3Mat22_zero;

// Identity matrix
extern const b3Mat22 b3Mat22_identity;

// Multiply a matrix times a vector.
inline b3Vec2 operator*(const b3Mat22& A, const b3Vec2& v)
{
	return v.x * A.x + v.y * A.y;
}

// Add two matrices.
inline b3Mat22 operator+(const b3Mat22& A, const b3Mat22& B)
{
	return b3Mat22(A.x + B.x, A.y + B.y);
}

// Subtract two matrices.
inline b3Mat22 operator-(const b3Mat22& A, const b3Mat22& B)
{
	return b3Mat22(A.x - B.x, A.y - B.y);
}

// Multiply two matrices.
inline b3Mat22 operator*(const b3Mat22& A, const b3Mat22& B)
{
	return b3Mat22(A * B.x, A * B.y);
}

// Multiply a scalar times a matrix.
inline b3Mat22 operator*(scalar s, const b3Mat22& A)
{
	return b3Mat22(s * A.x, s * A.y);
}

// Multiply a matrix times a vector.
inline b3Vec2 b3Mul(const b3Mat22& A, const b3Vec2& v)
{
	return v.x * A.x + v.y * A.y;
}

// Transpose a matrix.
inline b3Mat22 b3Transpose(const b3Mat22& A)
{
	return b3Mat22(
		b3Vec2(A.x.x, A.y.x),
		b3Vec2(A.x.y, A.y.y)
	);
}

// Invert a matrix.
// If the matrix determinant is zero this returns 
// the zero matrix.
b3Mat22 b3Inverse(const b3Mat22& A);

#endif