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

#ifndef B3_GEOMETRY_H
#define B3_GEOMETRY_H

#include <bounce/common/math/math.h>
#include <bounce/common/math/transform.h>

// A ray.
struct b3Ray3
{
	b3Vec3 A() const
	{
		return origin;
	}
	
	b3Vec3 B() const
	{
		return origin + fraction * direction;
	}

	b3Vec3 direction;
	b3Vec3 origin;
	float32 fraction;
};

// A plane in constant normal form.
// dot(n, p) - d = 0.
struct b3Plane
{
	// Does nothing for performance.
	b3Plane() { }
	
	// Set this plane from a normal and a signed distance from its origin.
	b3Plane(const b3Vec3& _normal, float32 _offset) 
	{
		normal = _normal;
		offset = _offset;
	}
	
	// Set this plane from a normal and a point on the plane.
	b3Plane(const b3Vec3& _normal, const b3Vec3& _point)
	{ 
		normal = _normal;
		offset = b3Dot(_normal, _point);
	}

	// Compute this plane from three non-colinear points.
	b3Plane(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
	{
		b3Vec3 N = b3Cross(B - A, C - A);
		normal = b3Normalize(N);
		offset = b3Dot(normal, A);
	}

	b3Vec3 normal;
	float32 offset;
};

// Transform a plane by a given frame.
inline b3Plane b3Mul(const b3Transform& T, const b3Plane& plane)
{
	b3Vec3 normal = b3Mul(T.rotation, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, T.position));
}

// Transform a plane by a given frame.
inline b3Plane operator*(const b3Transform& T, const b3Plane& plane)
{
	return b3Mul(T, plane);
}

// Compute the distance between a point and a plane.
inline float32 b3Distance(const b3Vec3& P, const b3Plane& plane)
{
	return b3Dot(plane.normal, P) - plane.offset;
}

// Project a point onto a normal plane.
inline b3Vec3 b3ClosestPointOnPlane(const b3Vec3& P, const b3Plane& plane)
{
	float32 fraction = b3Distance(P, plane);
	return P - fraction * plane.normal;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v) 
// with respect to a segment AB.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(float32 out[3], 
	const b3Vec3& A, const b3Vec3& B, 
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	
	float32 divisor = b3Dot(AB, AB);
	
	out[0] = b3Dot(QB, AB);
	out[1] = -b3Dot(QA, AB);
	out[2] = divisor;
}
// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w) 
// with respect to a triangle ABC.
// The last output value is the divisor.
inline void b3BarycentricCoordinates(float32 out[4],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	//float32 divisor = b3Dot(AB_x_AC, AB_x_AC);

	out[0] = b3Dot(QB_x_QC, AB_x_AC);
	out[1] = b3Dot(QC_x_QA, AB_x_AC);
	out[2] = b3Dot(QA_x_QB, AB_x_AC);
	out[3] = out[0] + out[1] + out[2];
}

// Project a point onto a segment AB.
inline b3Vec3 b3ClosestPointOnSegment(const b3Vec3& P, const b3Vec3& A, const b3Vec3& B)
{
	float32 wAB[3];
	b3BarycentricCoordinates(wAB, A, B, P);

	if (wAB[1] <= 0.0f)
	{
		return A;
	}

	if (wAB[0] <= 0.0f)
	{
		return B;
	}

	float32 s = 1.0f / wAB[2];
	float32 wA = s * wAB[0];
	float32 wB = s * wAB[1];
	return wA * A + wB * B;
}

#endif
