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

#ifndef B3_TRANSFORM_H
#define B3_TRANSFORM_H

#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>

// A transform represents a rigid frame. 
// It has a translation representing a position
// and a rotation representing an orientation.
struct b3Transform 
{
	// Default ctor does nothing for performance.
	b3Transform() { }
	
	// Set this transform from a translation vector and an orientation 
	// quaternion.
	b3Transform(const b3Vec3& p, const b3Quat& q)
	{
		position = p;
		rotation = b3ConvertQuatToMat(q);
	}
	
	// Set this transform to the identity.
	void SetIdentity() 
	{
		position.SetZero();
		rotation.SetIdentity();
	}

	b3Vec3 position; // in fact a translation
	b3Mat33 rotation;
};

// Motion proxy for TOI computation.
struct b3Sweep
{
	b3Vec3 localCenter; // local center
		
	b3Vec3 worldCenter0; // last world center
	b3Quat orientation0; // last orientation
	float32 t0; // last fraction between [0, 1]

	b3Vec3 worldCenter; // world center
	b3Quat orientation; // world orientation

	// Get this sweep transform at a given time between [0, 1]
	b3Transform GetTransform(float32 t) const;

	// Advance to a new initial state.
	void Advance(float32 t);
};

inline b3Transform b3Sweep::GetTransform(float32 t) const
{
	b3Vec3 c = (1.0f - t) * worldCenter0 + t * worldCenter;
	b3Quat q = (1.0f - t) * orientation0 + t * orientation;
	q.Normalize();

	b3Transform xf;
	xf.rotation = b3ConvertQuatToMat(q);
	xf.position = c - b3Mul(q, localCenter);
	return xf;
}

inline void b3Sweep::Advance(float32 t)
{
	B3_ASSERT(t < 1.0f);
	float32 dt = (t - t0) / (1.0f / t0);
	worldCenter += dt * (worldCenter - worldCenter0);
	orientation += dt * (orientation - orientation0);
	orientation.Normalize();
	t0 = t;
}

// Multiply a transform times a vector. If the transform 
// represents a frame this returns the vector in terms 
// of the frame.
inline b3Vec3 operator*(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T.rotation, v) + T.position;
}

// Multiply a transform times another transform (composed transform).
// [A y][B x] = [AB Ax+y]
// [0 1][0 1]   [0  1   ]
inline b3Transform operator*(const b3Transform& A, const b3Transform& B)
{
	b3Transform C;
	C.rotation = b3Mul(A.rotation, B.rotation);
	C.position = b3Mul(A.rotation, B.position) + A.position;
	return C;
}

// Multiply a transform times a vector.
inline b3Vec3 b3Mul(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T.rotation, v) + T.position;
}

// Multiply a transform times another transform.
// [A y][B x] = [AB Ax+y]
// [0 1][0 1]   [0  1   ]
inline b3Transform b3Mul(const b3Transform& A, const b3Transform& B)
{
	b3Transform C;
	C.rotation = b3Mul(A.rotation, B.rotation);
	C.position = b3Mul(A.rotation, B.position) + A.position;
	return C;
}

// Multiply the transpose of one transform (inverse 
// transform) times another transform (composed transform).
//[A^-1  -A^-1*y][B x] = [A^-1*B A^-1(x-y)]
//[0      1     ][0 1]   [0      1        ]
inline b3Transform b3MulT(const b3Transform& A, const b3Transform& B) 
{
	b3Transform C;
	C.rotation = b3MulT(A.rotation, B.rotation);
	C.position = b3MulT(A.rotation, B.position - A.position);
	return C;
}

// Multiply the transpose of a transform times a vector.
// If the transform represents a frame then this transforms
// the vector from one frame to another (inverse transform).
//[A^-1  -A^-1*y][x] = A^-1*x - A^-1*y = A^-1 * (x - y)
//[0     1      ][1]   
inline b3Vec3 b3MulT(const b3Transform& A, const b3Vec3& v)
{
	return b3MulT(A.rotation, v - A.position);
}

// Inverse transform.
inline b3Transform b3Inverse(const b3Transform& T)
{
	b3Transform B;
	B.rotation = b3Transpose(T.rotation);
	B.position = b3MulT(T.rotation, -T.position);
	return B;
}

#endif
