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
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>

static void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3CapsuleShape* s1,
	const b3Transform& xf2, u32 index2, const b3HullShape* s2)
{
	b3Vec3 P1 = xf1 * s1->m_centers[0];
	b3Vec3 Q1 = xf1 * s1->m_centers[1];
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	float32 L1 = N1.Normalize();
	B3_ASSERT(L1 > 0.0f);

	const b3Hull* hull2 = s2->m_hull;
	const b3HalfEdge* edge2 = hull2->GetEdge(index2);
	const b3HalfEdge* twin2 = hull2->GetEdge(index2 + 1);

	b3Vec3 C2 = xf2 * hull2->centroid;
	b3Vec3 P2 = xf2 * hull2->GetVertex(edge2->origin);
	b3Vec3 Q2 = xf2 * hull2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = E2;
	float32 L2 = N2.Normalize();
	B3_ASSERT(L2 > 0.0f);

	// Compute the closest points on the two lines.
	float32 b = b3Dot(N1, N2);
	float32 den = 1.0f - b * b;
	if (den <= 0.0f)
	{
		return;
	}

	float32 inv_den = 1.0f / den;

	b3Vec3 E3 = P1 - P2;

	float32 d = b3Dot(N1, E3);
	float32 e = b3Dot(N2, E3);

	float32 s = inv_den * (b * e - d);
	float32 t = inv_den * (e - b * d);

	b3Vec3 c1 = P1 + s * N1;
	b3Vec3 c2 = P2 + t * N2;

	// Ensure normal orientation to hull 2.
	b3Vec3 N = b3Cross(E1, E2);
	float32 LN = N.Normalize();
	B3_ASSERT(LN > 0.0f);
	if (b3Dot(N, P2 - C2) > 0.0f)
	{
		N = -N;
	}

	b3FeaturePair pair = b3MakePair(0, 1, index2, index2 + 1);
	
	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulT(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, c1);
	manifold.points[0].localPoint2 = b3MulT(xf2, c2);
	manifold.points[0].key = b3MakeKey(pair);
}

static void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3CapsuleShape* s1,
	const b3Transform& xf2, u32 index2, const b3HullShape* s2)
{
	// Clip edge 1 against the side planes of the face 2.
	const b3Capsule hull1(xf1 * s1->m_centers[0], xf1 * s1->m_centers[1], 0.0f);
	float32 r1 = s1->m_radius;

	b3ClipVertex edge1[2];
	b3BuildEdge(edge1, &hull1);
	
	const b3Hull* hull2 = s2->m_hull;
	float32 r2 = s2->m_radius;

	b3ClipVertex clipEdge1[2];
	u32 clipCount = b3ClipEdgeToFace(clipEdge1, edge1, xf2, r2, index2, hull2);

	// Project clipped edge 1 onto face plane 2.
	b3Plane localPlane2 = hull2->GetPlane(index2);
	b3Plane plane2 = xf2 * localPlane2;
	const b3Face* face2 = hull2->GetFace(index2);
	const b3HalfEdge* edge2 = hull2->GetEdge(face2->edge);
	b3Vec3 localPoint2 = hull2->GetVertex(edge2->origin);

	// Ensure normal orientation to hull 2.
	b3Vec3 n1 = -plane2.normal;

	float32 totalRadius = r1 + r2;

	u32 pointCount = 0;
	for (u32 i = 0; i < clipCount; ++i)
	{
		b3Vec3 c1 = clipEdge1[i].position;
		float32 s = b3Distance(c1, plane2);
		if (s <= totalRadius)
		{
			b3Vec3 c2 = b3ClosestPointOnPlane(c1, plane2);
			
			b3ManifoldPoint* mp = manifold.points + pointCount;
			mp->localNormal1 = b3MulT(xf1.rotation, n1);
			mp->localPoint1 = b3MulT(xf1, c1);
			mp->localPoint2 = b3MulT(xf2, c2);
			mp->key = b3MakeKey(clipEdge1[i].pair);
			
			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

void b3CollideCapsuleAndHull(b3Manifold& manifold, 
	const b3Transform& xf1, const b3CapsuleShape* s1,
	const b3Transform& xf2, const b3HullShape* s2)
{
	b3ShapeGJKProxy proxy1(s1, 0);
	b3ShapeGJKProxy proxy2(s2, 0);

	b3GJKOutput gjk = b3GJK(xf1, proxy1, xf2, proxy2);

	float32 r1 = s1->m_radius;
	float32 r2 = s2->m_radius;

	float32 totalRadius = r1 + r2;

	if (gjk.distance > totalRadius)
	{
		return;
	}

	const b3Capsule hull1(s1->m_centers[0], s1->m_centers[1], 0.0f);
	const b3Hull* hull2 = s2->m_hull;

	if (gjk.distance > B3_EPSILON)
	{
		// Define incident face.
		b3Vec3 N1 = (gjk.point2 - gjk.point1) / gjk.distance;
		b3Vec3 localN1 = b3MulT(xf2.rotation, N1);

		// Search reference face.
		u32 index2 = hull2->GetSupportFace(-localN1);
		b3Vec3 localN2 = hull2->GetPlane(index2).normal;
		b3Vec3 N2 = xf2.rotation * localN2;

		// Paralell vectors |v1xv2| = sin(theta)
		const float32 kTol = 0.005f;
		b3Vec3 N = b3Cross(N1, N2);
		float32 L = b3Dot(N, N);
		if (L < kTol * kTol)
		{
			// Reference face found.
			// Try to build a face contact.
			b3BuildFaceContact(manifold, xf1, s1, xf2, index2, s2);
			if (manifold.pointCount == 2)
			{
				return;
			}
		}
		
		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulT(xf1.rotation, N1);
		manifold.points[0].localPoint1 = b3MulT(xf1, gjk.point1);
		manifold.points[0].localPoint2 = b3MulT(xf2, gjk.point2);
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}
	
	b3FaceQuery faceQuery2 = b3QueryFaceSeparation(xf1, &hull1, xf2, hull2);
	if (faceQuery2.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, &hull1, xf2, hull2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	const float32 kTol = 0.1f * B3_LINEAR_SLOP;
	if (edgeQuery.separation > faceQuery2.separation + kTol)
	{
		b3BuildEdgeContact(manifold, xf1, s1, xf2, edgeQuery.index2, s2);
	}
	else
	{
		b3BuildFaceContact(manifold, xf1, s1, xf2, faceQuery2.index, s2);
	}
}