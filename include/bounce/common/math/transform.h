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
#include <bounce/common/math/mat44.h>
#include <bounce/common/math/quat.h>

// A transform represents a rigid frame. 
// It has a translation representing a position 
// and a rotation matrix representing an orientation 
// relative to some reference frame.
struct b3Transform 
{
	// Default ctor does nothing for performance.
	b3Transform() { }
	
	// Set this transform from a rotation matrix and a translation vector.
	b3Transform(const b3Mat33& _rotation, const b3Vec3& _translation)
	{
		rotation = _rotation;
		position = _translation;
	}
	
	// Set this transform from a rotation quaternion and a translation vector.
	b3Transform(const b3Quat& _rotation, const b3Vec3& _translation)
	{
		rotation = b3QuatMat33(_rotation);
		position = _translation;
	}
	
	// Set this transform to the identity transform.
	void SetIdentity() 
	{
		rotation.SetIdentity();
		position.SetZero();
	}

	b3Mat33 rotation;
	b3Vec3 position; // in fact a translation
};

// Identity transformation
extern const b3Transform b3Transform_identity;

// Convert a transform to a 4-by-4 transformation matrix. 
inline b3Mat44 b3TransformMat44(const b3Transform& T)
{
	return b3Mat44(
		b3Vec4(T.rotation.x.x, T.rotation.x.y, T.rotation.x.z, 0.0f),
		b3Vec4(T.rotation.y.x, T.rotation.y.y, T.rotation.y.z, 0.0f),
		b3Vec4(T.rotation.z.x, T.rotation.z.y, T.rotation.z.z, 0.0f),
		b3Vec4(T.position.x, T.position.y, T.position.z, 1.0f));
}

// Multiply a transform times a vector.
inline b3Vec3 b3Mul(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T.rotation, v) + T.position;
}

// Multiply a transform times another transform.
inline b3Transform b3Mul(const b3Transform& A, const b3Transform& B)
{
	// [A y][B x] = [AB Ax+y]
	// [0 1][0 1]   [0  1   ]
	b3Transform C;
	C.rotation = b3Mul(A.rotation, B.rotation);
	C.position = b3Mul(A.rotation, B.position) + A.position;
	return C;
}

// Multiply the transpose of one transform (inverse 
// transform) times another transform (composed transform).
inline b3Transform b3MulT(const b3Transform& A, const b3Transform& B) 
{
	//[A^-1  -A^-1*y][B x] = [A^-1*B A^-1(x-y)]
	//[0      1     ][0 1]   [0      1        ]
	b3Transform C;
	C.rotation = b3MulT(A.rotation, B.rotation);
	C.position = b3MulT(A.rotation, B.position - A.position);
	return C;
}

// Multiply the transpose of a transform times a vector.
// If the transform represents a frame then this transforms
// the vector from one frame to another (inverse transform).
inline b3Vec3 b3MulT(const b3Transform& A, const b3Vec3& v)
{
	//[A^-1  -A^-1*y][x] = A^-1*x - A^-1*y = A^-1 * (x - y)
	//[0     1      ][1]   
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

// Multiply a transform times a vector. If the transform 
// represents a frame this returns the vector in terms 
// of the frame.
inline b3Vec3 operator*(const b3Transform& T, const b3Vec3& v)
{
	return b3Mul(T, v);
}

// Multiply a transform times another transform (composed transform).
inline b3Transform operator*(const b3Transform& A, const b3Transform& B)
{
	return b3Mul(A, B);
}

// A quaternion-based transform.
struct b3TransformQT
{
	// Default ctor does nothing for performance.
	b3TransformQT() { }

	// Set this transform from a rotation matrix and a translation vector.
	b3TransformQT(const b3Mat33& _rotation, const b3Vec3& _translation)
	{
		rotation = b3Mat33Quat(_rotation);
		translation = _translation;
	}

	// Set this transform to the identity transform.
	void SetIdentity()
	{
		rotation.SetIdentity();
		translation.SetZero();
	}

	b3Quat rotation;
	b3Vec3 translation;
};

// Convert a quaternion based transform to a matrix based transform. 
inline b3Transform b3ConvertToTransform(const b3TransformQT& T)
{
	return b3Transform(T.rotation, T.translation);
}

// Multiply a transform times another transform.
inline b3TransformQT b3Mul(const b3TransformQT& A, const b3TransformQT& B)
{
	b3TransformQT C;
	C.rotation = b3Mul(A.rotation, B.rotation);
	C.translation = b3Mul(A.rotation, B.translation) + A.translation;
	return C;
}

// Multiply the transpose of one transform (inverse 
// transform) times another transform (composed transform).
inline b3TransformQT b3MulT(const b3TransformQT& A, const b3TransformQT& B)
{
	b3TransformQT C;
	C.rotation = b3MulT(A.rotation, B.rotation);
	C.translation = b3MulT(A.rotation, B.translation - A.translation);
	return C;
}

inline b3TransformQT operator*(const b3TransformQT& A, const b3TransformQT& B)
{
	return b3Mul(A, B);
}

// Inverse transform a vector.
inline b3Vec3 b3MulT(const b3TransformQT& A, const b3Vec3& v)
{
	return b3MulT(A.rotation, v - A.translation);
}

// Inverse transform.
inline b3TransformQT b3Inverse(const b3TransformQT& T)
{
	b3TransformQT B;
	B.rotation = b3Conjugate(T.rotation);
	B.translation = b3MulT(T.rotation, -T.translation);
	return B;
}

// Motion proxy for TOI computation.
struct b3Sweep
{
	// Get this sweep transform at a given time between [0, 1]
	b3Transform GetTransform(float32 t) const;

	// Advance to a new initial state.
	void Advance(float32 t);

	b3Vec3 localCenter; // local center

	b3Quat orientation0; // last orientation
	b3Vec3 worldCenter0; // last world center
	
	float32 t0; // last fraction between [0, 1]

	b3Quat orientation; // world orientation
	b3Vec3 worldCenter; // world center
};

inline b3Transform b3Sweep::GetTransform(float32 t) const
{
	b3Vec3 c = (1.0f - t) * worldCenter0 + t * worldCenter;
	b3Quat q = (1.0f - t) * orientation0 + t * orientation;
	q.Normalize();

	b3Transform xf;
	xf.rotation = b3QuatMat33(q);
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

#endif