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

#include <bounce\dynamics\contacts\collide\collide.h>
#include <bounce\dynamics\contacts\collide\clip.h>
#include <bounce\dynamics\contacts\manifold.h>
#include <bounce\dynamics\shapes\capsule_shape.h>
#include <bounce\collision\shapes\capsule.h>

bool b3AreParalell(const b3Capsule& hullA, const b3Capsule& hullB)
{
	b3Vec3 E1 = hullA.vertices[1] - hullA.vertices[0];
	float32 L1 = b3Length(E1);
	if (L1 < B3_LINEAR_SLOP)
	{
		return false;
	}

	b3Vec3 E2 = hullB.vertices[1] - hullB.vertices[0];
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
	const b3Transform& xfA, const b3CapsuleShape* sA,
	const b3Transform& xfB, const b3CapsuleShape* sB)
{
	b3Capsule hullA;
	hullA.vertices[0] = b3Mul(xfA, sA->m_centers[0]);
	hullA.vertices[1] = b3Mul(xfA, sA->m_centers[1]);
	hullA.radius = sA->m_radius;

	b3Capsule hullB;
	hullB.vertices[0] = b3Mul(xfB, sB->m_centers[0]);
	hullB.vertices[1] = b3Mul(xfB, sB->m_centers[1]);
	hullB.radius = sB->m_radius;

	float32 totalRadius = hullA.radius + hullB.radius;

	if (b3AreParalell(hullA, hullB))
	{
		// Clip edge A against the side planes of edge B.
		b3ClipVertex edgeA[2];
		b3BuildEdge(edgeA, &hullA);

		b3ClipVertex clipEdgeA[2];
		u32 clipCount = b3ClipEdgeToFace(clipEdgeA, edgeA, &hullB);

		float32 totalRadius = hullA.radius + hullB.radius;

		if (clipCount == 2)
		{
			b3Vec3 cp1 = b3ClosestPointOnSegment(clipEdgeA[0].position, hullB.vertices[0], hullB.vertices[1]);
			b3Vec3 cp2 = b3ClosestPointOnSegment(clipEdgeA[1].position, hullB.vertices[0], hullB.vertices[1]);

			float32 d1 = b3Distance(clipEdgeA[0].position, cp1);
			float32 d2 = b3Distance(clipEdgeA[1].position, cp2);

			if (d1 <= totalRadius && d2 <= totalRadius)
			{
				b3Vec3 n1 = (cp1 - clipEdgeA[0].position) / d1;
				b3Vec3 n2 = (cp2 - clipEdgeA[1].position) / d2;

				b3Vec3 p1 = 0.5f * (clipEdgeA[0].position + hullA.radius * n1 + cp1 - hullB.radius * n1);
				b3Vec3 p2 = 0.5f * (clipEdgeA[1].position + hullA.radius * n2 + cp2 - hullB.radius * n2);

				b3Vec3 center = 0.5f * (p1 + p2);
				b3Vec3 normal = b3Normalize(n1 + n2);

				manifold.pointCount = 2;

				manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
				manifold.points[0].key = b3MakeKey(clipEdgeA[0].pair);
				manifold.points[0].localNormal = b3MulT(xfA.rotation, n1);
				manifold.points[0].localPoint = b3MulT(xfA, clipEdgeA[0].position);
				manifold.points[0].localPoint2 = b3MulT(xfB, cp1);

				manifold.points[1].triangleKey = B3_NULL_TRIANGLE;
				manifold.points[1].key = b3MakeKey(clipEdgeA[1].pair);
				manifold.points[1].localNormal = b3MulT(xfA.rotation, n2);
				manifold.points[1].localPoint = b3MulT(xfA, clipEdgeA[1].position);
				manifold.points[1].localPoint2 = b3MulT(xfB, cp2);

				manifold.center = center;
				manifold.normal = normal;
				manifold.tangent1 = b3Perp(normal);
				manifold.tangent2 = b3Cross(manifold.tangent1, normal);

				return;
			}
		}
	}

	b3Vec3 pointA, pointB;
	b3ClosestPointsOnSegments(&pointA, &pointB, hullA.vertices[0], hullA.vertices[1], hullB.vertices[0], hullB.vertices[1]);
	if (b3DistanceSquared(pointA, pointB) > totalRadius * totalRadius)
	{
		return;
	}

	float32 distance = b3Distance(pointA, pointB);

	if (distance > 0.0f)
	{
		b3Vec3 normal = (pointB - pointA) / distance;
		b3Vec3 center = 0.5f * (pointA + hullA.radius * normal + pointB - hullB.radius * normal);

		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = 0;
		manifold.points[0].localNormal = b3MulT(xfA.rotation, normal);
		manifold.points[0].localPoint = b3MulT(xfA, pointA);
		manifold.points[0].localPoint2 = b3MulT(xfB, pointB);
		
		manifold.center = center;
		manifold.normal = normal;
		manifold.tangent1 = b3Perp(normal);
		manifold.tangent2 = b3Cross(manifold.tangent1, normal);
	}
}
