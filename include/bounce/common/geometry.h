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

// A triangle in indexed form.
struct b3Triangle
{
	// Does nothing for performance.
	b3Triangle() { }

	// Set this triangle from three vertices.
	b3Triangle(u32 _v1, u32 _v2, u32 _v3) 
	{
		v1 = _v1;
		v2 = _v2;
		v3 = _v3;
	}
	
	// Set this triangle from three vertices.
	void Set(u32 _v1, u32 _v2, u32 _v3)
	{
		v1 = _v1;
		v2 = _v2;
		v3 = _v3;
	}

	// Test if this triangle contains a given vertex.
	bool TestVertex(u32 v) const
	{
		return v == v1 || v == v2 || v == v3;
	}

	// Test if this triangle contains two vertices.
	bool TestEdge(u32 _v1, u32 _v2) const
	{
		return TestVertex(_v1) && TestVertex(_v2);
	}

	u32 v1, v2, v3;
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
inline b3Plane operator*(const b3Transform& T, const b3Plane& plane)
{
	b3Vec3 normal = b3Mul(T.rotation, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, T.position));
}

// Transform a plane by a given frame.
inline b3Plane b3Mul(const b3Transform& T, const b3Plane& plane)
{
	b3Vec3 normal = b3Mul(T.rotation, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, T.position));
}

// Compute the distance between a point and a plane.
inline float32 b3Distance(const b3Vec3& P, const b3Plane& plane)
{
	return b3Dot(plane.normal, P) - plane.offset;
}

// Project a point onto a normal plane.
inline b3Vec3 b3Project(const b3Vec3& P, const b3Plane& plane)
{
	float32 fraction = b3Distance(P, plane);
	return P - fraction * plane.normal;
}

// Compute barycentric coordinates (u, v) for point Q to segment AB.
// The last output value is the divisor.
inline void b3Barycentric(float32 out[3], 
	const b3Vec3& A, const b3Vec3& B, 
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	//float32 divisor = b3Dot(AB, AB);
	out[0] = b3Dot(QB, AB);
	out[1] = -b3Dot(QA, AB);
	out[2] = out[0] + out[1];
}

// Compute barycentric coordinates (u, v, w) for point Q to triangle ABC.
// The last output value is the divisor.
inline void b3Barycentric(float32 out[4], 
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, 
	const b3Vec3& Q)
{
	// RTCD, 140.
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

// Compute barycentric coordinates (u, v, w, x) for point Q to tetrahedron ABCD.
// The last output value is the (positive) divisor.
inline void b3Barycentric(float32 out[5], 
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D, 
	const b3Vec3& Q)
{
	// RTCD, 48, 49.
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;
	b3Vec3 AD = D - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;
	b3Vec3 QD = D - Q;

	float32 divisor = b3Det(AB, AC, AD);
	float32 sign = b3Sign(divisor);

	out[0] = sign * b3Det(QB, QC, QD);
	out[1] = sign * b3Det(QA, QD, QC);
	out[2] = sign * b3Det(QA, QB, QD);
	out[3] = sign * b3Det(QA, QC, QB);
	out[4] = sign * divisor;
}

#endif
