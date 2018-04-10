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

#ifndef B3_MAT_33_H
#define B3_MAT_33_H

#include <bounce/common/math/vec3.h>

// A 3-by-3 matrix stored in column-major order.
struct b3Mat33 
{
	// Does nothing for performance.
	b3Mat33() { }

	// Set this matrix from three column vectors.
	b3Mat33(const b3Vec3& _x, const b3Vec3& _y, const b3Vec3& _z) :	x(_x), y(_y), z(_z) { }

	// Read an indexed column vector from this matrix.
	const b3Vec3& operator[](u32 i) const 
	{
		return (&x)[i];
	}
	
	// Write an indexed column vector to this matrix.
	b3Vec3& operator[](u32 i) 
	{
		return (&x)[i];
	}

	// Read an indexed element from this matrix.
	float32 operator()(u32 i, u32 j) const
	{
		return (&x.x)[i + 3 * j];
	}

	// Add a matrix to this matrix.
	void operator+=(const b3Mat33& B)
	{
		x += B.x;
		y += B.y;
		z += B.z;
	}

	// Subtract this matrix from a matrix.
	void operator-=(const b3Mat33& B)
	{
		x -= B.x;
		y -= B.y;
		z -= B.z;
	}
	
	// Set this matrix to the zero matrix.
	void SetZero() 
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
	}

	// Set this matrix to the identity matrix.
	void SetIdentity() 
	{
		x.Set(1.0f, 0.0f, 0.0f);
		y.Set(0.0f, 1.0f, 0.0f);
		z.Set(0.0f, 0.0f, 1.0f);
	}

	// Solve Ax = b. 
	// It doesn't compute the inverse. 
	// Therefore, is more efficient.
	// Returns the zero vector if the matrix is singular.
	b3Vec3 Solve(const b3Vec3& b) const;

	b3Vec3 x, y, z;
};

// Usefull constants.
extern const b3Mat33 b3Mat33_zero;
extern const b3Mat33 b3Mat33_identity;

// Add two matrices.
inline b3Mat33 operator+(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A.x + B.x, A.y + B.y, A.z + B.z);
}

// Subtract two matrices.
inline b3Mat33 operator-(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A.x - B.x, A.y - B.y, A.z - B.z);
}

// Multiply a scalar times a matrix.
inline b3Mat33 operator*(float32 s, const b3Mat33& A) 
{
	return b3Mat33(s * A.x, s * A.y, s * A.z);
}

// Negate a matrix.
inline b3Mat33 operator-(const b3Mat33& A)
{
	return -1.0f * A;
}

// Multiply a matrix times a vector. If the matrix 
// represents a rotation this transforms the vector 
// from one frame to another.
inline b3Vec3 operator*(const b3Mat33& A, const b3Vec3& v) 
{
	return v.x * A.x + v.y * A.y + v.z * A.z;
}

// Multiply two matrices.
inline b3Mat33 operator*(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(A * B.x, A * B.y, A * B.z);
}

// Multiply a matrix times a vector. If the matrix 
// represents a rotation this transforms the vector 
// from one frame to another.
inline b3Vec3 b3Mul(const b3Mat33& A, const b3Vec3& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z;
}

// Multiply two matrices.
inline b3Mat33 b3Mul(const b3Mat33& A, const b3Mat33& B)
{
	return b3Mat33( b3Mul(A, B.x), b3Mul(A, B.y), b3Mul(A, B.z));
}

// Multiply the transpose of a matrix times a vector. If 
// the matrix represents a rotation frame this transforms the 
// vector from one frame to another (inverse transform).
inline b3Vec3 b3MulT(const b3Mat33& A, const b3Vec3& v) 
{
	return b3Vec3(b3Dot(A.x, v), b3Dot(A.y, v), b3Dot(A.z, v));
}

// Multiply the transpose of a matrix times another.
inline b3Mat33 b3MulT(const b3Mat33& A, const b3Mat33& B) 
{
	return b3Mat33(
		b3Vec3(b3Dot(A.x, B.x), b3Dot(A.y, B.x), b3Dot(A.z, B.x)),
		b3Vec3(b3Dot(A.x, B.y), b3Dot(A.y, B.y), b3Dot(A.z, B.y)),
		b3Vec3(b3Dot(A.x, B.z), b3Dot(A.y, B.z), b3Dot(A.z, B.z)));
}

// Transpose a matrix.
inline b3Mat33 b3Transpose(const b3Mat33& A) 
{
	return b3Mat33(
		b3Vec3(A.x.x, A.y.x, A.z.x),
		b3Vec3(A.x.y, A.y.y, A.z.y),
		b3Vec3(A.x.z, A.y.z, A.z.z)
		);
}

// Uniform scale matrix.
inline b3Mat33 b3Diagonal(float32 s) 
{
	return b3Mat33(
		b3Vec3(s, 0.0f, 0.0f),
		b3Vec3(0.0f, s, 0.0f),
		b3Vec3(0.0f, 0.0f, s)
		);
}

// Uniform or non-uniform scale matrix.
inline b3Mat33 b3Diagonal(float32 x, float32 y, float32 z)
{
	return b3Mat33(
		b3Vec3(x, 0.0f, 0.0f),
		b3Vec3(0.0f, y, 0.0f),
		b3Vec3(0.0f, 0.0f, z)
		);
}

// Invert a matrix.
// If the matrix is singular this 
// returns the zero matrix.
b3Mat33 b3Inverse(const b3Mat33& A);

// Invert a symmetric matrix.
// If the matrix is singular this 
// returns the zero matrix.
b3Mat33 b3SymInverse(const b3Mat33& A);

// Return a skew (anti-symmetric) matrix for a vector.
inline b3Mat33 b3Skew(const b3Vec3& v) 
{
	return b3Mat33(
		b3Vec3(0.0f, v.z, -v.y),
		b3Vec3(-v.z, 0.0f, v.x),
		b3Vec3(v.y, -v.x, 0.0f)
		);
}

// Compute the dot product of two vectors.
inline float32 b3Inner(const b3Vec3& a, const b3Vec3& b)
{
	return b3Dot(a, b);
}

// Compute the outer product of two vectors.
// The result is a matrix A = a * b^T.
inline b3Mat33 b3Outer(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Mat33(b.x * a, b.y * a, b.z * a);
}

// Compute an orthogonal basis given one of its vectors.
// The vector must be normalized.
inline b3Mat33 b3Basis(const b3Vec3& a)
{
	// Box2D
	b3Mat33 A;
	if (b3Abs(a.x) >= float32(0.57735027))
	{
		A.y.Set(a.y, -a.x, 0.0f);
	}
	else
	{
		A.y.Set(0.0f, a.z, -a.y);
	}
	A.x = a;
	A.y = b3Normalize(A.y);
	A.z = b3Cross(a, A.y);
	return A;
}

// Rotation about the x-axis.
inline b3Mat33 b3Mat33RotationX(float32 angle)
{
	float32 c = cos(angle);
	float32 s = sin(angle);

	b3Mat33 R;
	R.x.Set(1.0f, 0.0f, 0.0f);
	R.y.Set(0.0f, c, s);
	R.z.Set(0.0f, -s, c);
	return R;
}

// Rotation about the y-axis.
inline b3Mat33 b3Mat33RotationY(float32 angle)
{
	float32 c = cos(angle);
	float32 s = sin(angle);

	b3Mat33 R;
	R.x.Set(c, 0.0f, -s);
	R.y.Set(0.0f, 1.0f, 0.0f);
	R.z.Set(s, 0.0f, c);
	return R;
}

// Rotation about the z-axis.
inline b3Mat33 b3Mat33RotationZ(float32 angle)
{
	float32 c = cos(angle);
	float32 s = sin(angle);

	b3Mat33 R;
	R.x.Set(c, s, 0.0f);
	R.y.Set(-s, c, 0.0f);
	R.z.Set(0.0f, 0.0f, 1.0f);
	return R;
}

#endif