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

#include <bounce/dynamics/contacts/collide/collide.h>
#include <bounce/dynamics/contacts/collide/clip.h>
#include <bounce/dynamics/contacts/manifold.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/common/math/mat22.h>

// Compute the closest point on a segment to a point. 
static b3Vec3 b3ClosestPointOnSegment(const b3Vec3& Q, const b3Capsule& hull)
{
	b3Vec3 A = hull.vertices[0];
	b3Vec3 B = hull.vertices[1];
	b3Vec3 AB = B - A;
	
	// Barycentric coordinates for Q
	float32 u = b3Dot(B - Q, AB);
	float32 v = b3Dot(Q - A, AB);
	
	if (v <= 0.0f)
	{
		return A;
	}

	if (u <= 0.0f)
	{
		return B;
	}

	float32 w = b3Dot(AB, AB);
	if (w <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		return A;
	}

	float32 den = 1.0f / w;
	b3Vec3 P = den * (u * A + v * B);
	return P;
}

// Compute the closest points between two line segments.
static void b3ClosestPoints(b3Vec3& C1, b3Vec3& C2,
	const b3Capsule& hull1, const b3Capsule& hull2)
{
	b3Vec3 P1 = hull1.vertices[0];
	b3Vec3 Q1 = hull1.vertices[1];
	
	b3Vec3 P2 = hull2.vertices[0];
	b3Vec3 Q2 = hull2.vertices[1];

	b3Vec3 E1 = Q1 - P1;
	float32 L1 = b3Length(E1);

	b3Vec3 E2 = Q2 - P2;
	float32 L2 = b3Length(E2);

	if (L1 < B3_LINEAR_SLOP && L2 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = P2;
		return;
	}

	if (L1 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = b3ClosestPointOnSegment(P1, hull2);
		return;
	}

	if (L2 < B3_LINEAR_SLOP)
	{
		C1 = b3ClosestPointOnSegment(P2, hull1);
		C2 = P2;
		return;
	}
	
	B3_ASSERT(L1 > 0.0f);
	b3Vec3 N1 = E1 / L1;
	
	B3_ASSERT(L2 > 0.0f);
	b3Vec3 N2 = E2 / L2;
	
	// [1 -dot(n1, n2)][x1] = [-dot(n1, P1 - P2)] 
	// [dot(n2, n1) -1][x2] = [-dot(n2, P1 - P2)]
	// Ax = b
	float32 a11 = 1.0f, a12 = -b3Dot(N1, N2);
	float32 a21 = -a12, a22 = -1.0f;
	
	b3Mat22 A;
	A.x.Set(a11, a21);
	A.y.Set(a12, a22);

	float32 norm_A = b3Max(b3Abs(A.x.x) + b3Abs(A.x.y), b3Abs(A.y.x) + b3Abs(A.y.y));
	
	float32 det_A = 1.0f / (a11 * a22 - a12 * a21);

	b3Mat22 inv_A;
	inv_A.x.Set(det_A * a22, -det_A * a21);
	inv_A.y.Set(-det_A * a12, det_A * a11);

	float32 norm_inv_A = b3Max(b3Abs(inv_A.x.x) + b3Abs(inv_A.x.y), b3Abs(inv_A.y.x) + b3Abs(inv_A.y.y));

	float32 k_A = norm_A * norm_inv_A;

	const float32 kMaxConditionNumber = 1000.0f;
	
	// Ensure a reasonable condition number.
	if (b3IsInf(k_A) == false && k_A < kMaxConditionNumber)
	{
		// A is safe to invert.
		b3Vec3 E3 = P1 - P2;

		b3Vec2 b;
		b.x = -b3Dot(N1, E3);
		b.y = -b3Dot(N2, E3);

		b3Vec2 x = inv_A * b;

		C1 = P1 + x.x * N1;
		C2 = P2 + x.y * N2;
	}
	else
	{
		// The lines are intersecting.
		C1 = P1;
		C2 = P2;
	}

	C1 = b3ClosestPointOnSegment(C1, hull1);
	
	C2 = b3ClosestPointOnSegment(C1, hull2);
	
	C1 = b3ClosestPointOnSegment(C2, hull1);
}

static bool b3AreParalell(const b3Capsule& hull1, const b3Capsule& hull2)
{
	b3Vec3 E1 = hull1.vertices[1] - hull1.vertices[0];
	float32 L1 = b3Length(E1);
	if (L1 < B3_LINEAR_SLOP)
	{
		return false;
	}

	b3Vec3 E2 = hull2.vertices[1] - hull2.vertices[0];
	float32 L2 = b3Length(E2);
	if (L2 < B3_LINEAR_SLOP)
	{
		return false;
	}

	// |e1 x e2| = sin(theta) * |e1| * |e2|
	const float32 kTol = 0.005f;
	b3Vec3 N = b3Cross(E1, E2);
	return b3Length(N) < kTol * L1 * L2;
}

void b3CollideCapsuleAndCapsule(b3Manifold& manifold, 
	const b3Transform& xf1, const b3CapsuleShape* s1,
	const b3Transform& xf2, const b3CapsuleShape* s2)
{
	b3Capsule hull1;
	hull1.vertices[0] = xf1 * s1->m_centers[0];
	hull1.vertices[1] = xf1 * s1->m_centers[1];
	
	b3Capsule hull2;
	hull2.vertices[0] = xf2 * s2->m_centers[0];
	hull2.vertices[1] = xf2 * s2->m_centers[1];
	
	b3Vec3 point1, point2;
	b3ClosestPoints(point1, point2, hull1, hull2);
	
	float32 distance = b3Distance(point1, point2);

	float32 r1 = s1->m_radius;
	float32 r2 = s2->m_radius;
	float32 totalRadius = r1 + r2;
	if (distance > totalRadius)
	{
		return;
	}

	if (distance > B3_EPSILON)
	{
		if (b3AreParalell(hull1, hull2))
		{
			// Clip edge 1 against the side planes of edge 2.
			b3ClipVertex edge1[2];
			b3BuildEdge(edge1, &hull1);

			b3ClipVertex clipEdge1[2];
			u32 clipCount = b3ClipEdgeToFace(clipEdge1, edge1, &hull2);

			if (clipCount == 2)
			{
				b3Vec3 cp1 = b3ClosestPointOnSegment(clipEdge1[0].position, hull2);
				b3Vec3 cp2 = b3ClosestPointOnSegment(clipEdge1[1].position, hull2);

				float32 d1 = b3Distance(clipEdge1[0].position, cp1);
				float32 d2 = b3Distance(clipEdge1[1].position, cp2);

				if (d1 > B3_EPSILON && d1 <= totalRadius && d2 > B3_EPSILON && d2 <= totalRadius)
				{
					b3Vec3 n1 = (cp1 - clipEdge1[0].position) / d1;
					b3Vec3 n2 = (cp2 - clipEdge1[1].position) / d2;

					b3Vec3 p1 = 0.5f * (clipEdge1[0].position + r1 * n1 + cp1 - r2 * n1);
					b3Vec3 p2 = 0.5f * (clipEdge1[1].position + r1 * n2 + cp2 - r2 * n2);

					b3Vec3 center = 0.5f * (p1 + p2);
					b3Vec3 normal = b3Normalize(n1 + n2);

					manifold.pointCount = 2;

					manifold.points[0].localNormal1 = b3MulT(xf1.rotation, n1);
					manifold.points[0].localPoint1 = b3MulT(xf1, clipEdge1[0].position);
					manifold.points[0].localPoint2 = b3MulT(xf2, cp1);
					manifold.points[0].key = b3MakeKey(clipEdge1[0].pair);

					manifold.points[1].localNormal1 = b3MulT(xf1.rotation, n2);
					manifold.points[1].localPoint1 = b3MulT(xf1, clipEdge1[1].position);
					manifold.points[1].localPoint2 = b3MulT(xf2, cp2);
					manifold.points[1].key = b3MakeKey(clipEdge1[1].pair);

					return;
				}
			}
		}

		b3Vec3 normal = (point2 - point1) / distance;
		
		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulT(xf1.rotation, normal);
		manifold.points[0].localPoint1 = b3MulT(xf1, point1);
		manifold.points[0].localPoint2 = b3MulT(xf2, point2);
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}
}