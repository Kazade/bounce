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
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>

void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3Capsule* hull1,
	const b3Transform& xf2, u32 index2, const b3Hull* hull2)
{
	b3Vec3 P1 = b3Mul(xf1, hull1->GetVertex(0));
	b3Vec3 Q1 = b3Mul(xf1, hull1->GetVertex(1));
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = b3Normalize(E1);
	
	const b3HalfEdge* edge2 = hull2->GetEdge(index2);
	const b3HalfEdge* twin2 = hull2->GetEdge(index2 + 1);

	b3Vec3 C2 = b3Mul(xf2, hull2->centroid);
	b3Vec3 P2 = b3Mul(xf2, hull2->GetVertex(edge2->origin));
	b3Vec3 Q2 = b3Mul(xf2, hull2->GetVertex(twin2->origin));
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = b3Normalize(E2);

	b3Vec3 N = b3Cross(E1, E2);
	N.Normalize();
	if (b3Dot(N, P2 - C2) > 0.0f)
	{
		N = -N;
	}

	b3Vec3 PA, PB;
	b3ClosestPointsOnNormalizedLines(&PA, &PB, P1, N1, P2, N2);

	b3FeaturePair pair = b3MakePair(0, 1, index2, index2 + 1);
	
	manifold.pointCount = 1;
	manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key = b3MakeKey(pair);
	manifold.points[0].localNormal = b3MulT(xf1.rotation, N);
	manifold.points[0].localPoint = b3MulT(xf1, PA);
	manifold.points[0].localPoint2 = b3MulT(xf2, PB);

	manifold.center = 0.5f * (PA + hull1->radius * N + PB - B3_HULL_RADIUS * N);
	manifold.normal = N;
	manifold.tangent1 = b3Perp(N);
	manifold.tangent2 = b3Cross(manifold.tangent1, N);
}

void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3Capsule* hull1,
	const b3Transform& xf2, u32 index2, const b3Hull* hull2)
{
	// Clip edge 1 against the side planes of the face 2.
	b3Capsule tempHull1;
	tempHull1.vertices[0] = b3Mul(xf1, hull1->vertices[0]);
	tempHull1.vertices[1] = b3Mul(xf1, hull1->vertices[1]);
	tempHull1.radius = hull1->radius;

	b3ClipVertex edge1[2];
	b3BuildEdge(edge1, &tempHull1);

	b3ClipVertex clipEdge1[2];
	u32 clipCount = b3ClipEdgeToFace(clipEdge1, edge1, xf2, index2, hull2);

	// Project clipped edge 1 onto face plane 2.
	float32 r1 = hull1->radius;
	float32 r2 = B3_HULL_RADIUS;
	float32 totalRadius = r1 + r2;

	b3Plane localPlane2 = hull2->GetPlane(index2);
	b3Plane plane2 = b3Mul(xf2, localPlane2);
	const b3Face* face2 = hull2->GetFace(index2);
	const b3HalfEdge* edge2 = hull2->GetEdge(face2->edge);
	b3Vec3 localPoint2 = hull2->GetVertex(edge2->origin);

	b3Vec3 normal = -plane2.normal;
	
	b3Vec3 center;
	center.SetZero();

	u32 pointCount = 0;
	for (u32 i = 0; i < clipCount; ++i)
	{
		float32 s = b3Distance(clipEdge1[i].position, plane2);
		if (s <= totalRadius)
		{
			b3Vec3 cp = b3ClosestPointOnPlane(clipEdge1[i].position, plane2);
			b3Vec3 p = 0.5f * (clipEdge1[i].position + r1 * normal + cp - r2 * normal);

			b3ManifoldPoint* mp = manifold.points + pointCount;
			mp->triangleKey = B3_NULL_TRIANGLE;
			mp->key = b3MakeKey(clipEdge1[i].pair);
			mp->localNormal = b3MulT(xf1.rotation, normal);
			mp->localPoint = b3MulT(xf1, clipEdge1[i].position);
			mp->localPoint2 = b3MulT(xf2, cp);
			
			++pointCount;

			center += p;
		}
	}

	if (pointCount > 0)
	{
		center /= pointCount;
		
		manifold.center = center;
		manifold.normal = normal;
		manifold.tangent1 = b3Perp(normal);
		manifold.tangent2 = b3Cross(manifold.tangent1, normal);
		manifold.pointCount = pointCount;
	}
}

void b3CollideCapsuleAndHull(b3Manifold& manifold,
	const b3Transform& xfA, const b3CapsuleShape* sA,
	const b3Transform& xfB, const b3HullShape* sB)
{
	b3Capsule hullA;
	hullA.vertices[0] = sA->m_centers[0];
	hullA.vertices[1] = sA->m_centers[1];
	hullA.radius = sA->m_radius;
	const b3Hull* hullB = sB->m_hull;

	b3ShapeGJKProxy proxyA(sA, 0);
	b3ShapeGJKProxy proxyB(sB, 0);
	b3GJKOutput distance = b3GJK(xfA, proxyA, xfB, proxyB);
	float32 totalRadius = hullA.radius + B3_HULL_RADIUS;
	if (distance.distance > totalRadius)
	{
		return;
	}

	if (distance.distance > 0.0f)
	{
		// Define incident face.
		b3Vec3 NA = (distance.pointB - distance.pointA) / distance.distance;
		b3Vec3 localNA = b3MulT(xfB.rotation, NA);

		// Search reference face.
		u32 indexB = hullB->GetSupportFace(-localNA);
		b3Vec3 localNB = hullB->GetPlane(indexB).normal;
		b3Vec3 NB = b3Mul(xfB.rotation, localNB);

		// Paralell vectors |v1xv2| = sin(theta)
		const float32 kTol = 0.005f;
		b3Vec3 axis = b3Cross(NA, NB);
		float32 L = b3Dot(axis, axis);
		if (L < kTol * kTol)
		{
			// Reference face found.
			// Try to build a face contact.
			b3BuildFaceContact(manifold, xfA, &hullA, xfB, indexB, hullB);
			if (manifold.pointCount == 2)
			{
				return;
			}
		}
		
		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = 0;
		manifold.points[0].localNormal = b3MulT(xfA.rotation, NA);
		manifold.points[0].localPoint = b3MulT(xfA, distance.pointA);
		manifold.points[0].localPoint2 = b3MulT(xfB, distance.pointB);

		manifold.center = 0.5f * (distance.pointA + sA->m_radius * NA + distance.pointB - B3_HULL_RADIUS * NA);
		manifold.normal = NA;
		manifold.tangent1 = b3Perp(NA);
		manifold.tangent2 = b3Cross(manifold.tangent1, NA);

		return;
	}

	b3FaceQuery faceQueryB = b3QueryFaceSeparation(xfA, &hullA, xfB, hullB);
	if (faceQueryB.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xfA, &hullA, xfB, hullB);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	// Now the hulls are overlapping.
	const float32 kRelEdgeTol = 0.95f;
	const float32 kAbsTol = 0.05f;
	if (edgeQuery.separation > kRelEdgeTol * faceQueryB.separation + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xfA, &hullA, xfB, edgeQuery.indexB, hullB);
	}
	else
	{
		b3BuildFaceContact(manifold, xfA, &hullA, xfB, faceQueryB.index, hullB);
	}
}
