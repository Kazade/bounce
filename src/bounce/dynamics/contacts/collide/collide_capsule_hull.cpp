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

	b3Vec3 E3 = P1 - P2;

	b3Vec2 b;
	b.x = -b3Dot(N1, E3);
	b.y = -b3Dot(N2, E3);

	float32 a12 = -b3Dot(N1, N2), a21 = -a12;

	float32 det = -1.0f - a12 * a21;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	b3Vec2 x;
	x.x = det * (-b.x - a12 * b.y);
	x.y = det * (b.y - a21 * b.x);

	b3Vec3 point1 = P1 + x.x * N1;
	b3Vec3 point2 = P2 + x.y * N2;

	b3Vec3 axis = b3Cross(E1, E2);
	b3Vec3 normal = b3Normalize(axis);
	if (b3Dot(normal, P2 - C2) > 0.0f)
	{
		normal = -normal;
	}

	b3FeaturePair pair = b3MakePair(0, 1, index2, index2 + 1);
	
	manifold.pointCount = 1;
	manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key = b3MakeKey(pair);
	manifold.points[0].localNormal = b3MulT(xf1.rotation, normal);
	manifold.points[0].localPoint = b3MulT(xf1, point1);
	manifold.points[0].localPoint2 = b3MulT(xf2, point2);

	manifold.center = 0.5f * (point1 + hull1->radius * normal + point2 - B3_HULL_RADIUS * normal);
	manifold.normal = normal;
	manifold.tangent1 = b3Perp(normal);
	manifold.tangent2 = b3Cross(manifold.tangent1, normal);
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
	
	// Project.
	if (clipCount == 2)
	{
		float32 r1 = hull1->radius;
		float32 r2 = B3_HULL_RADIUS;
		float32 totalRadius = r1 + r2;

		b3Plane localPlane2 = hull2->GetPlane(index2);
		b3Plane plane2 = b3Mul(xf2, localPlane2);
		const b3Face* face2 = hull2->GetFace(index2);
		const b3HalfEdge* edge2 = hull2->GetEdge(face2->edge);
		b3Vec3 localPoint2 = hull2->GetVertex(edge2->origin);

		b3Vec3 cp1 = b3ClosestPointOnPlane(clipEdge1[0].position, plane2);
		b3Vec3 cp2 = b3ClosestPointOnPlane(clipEdge1[1].position, plane2);

		float32 s1 = b3Distance(clipEdge1[0].position, plane2);
		float32 s2 = b3Distance(clipEdge1[1].position, plane2);

		if (s1 <= totalRadius && s2 <= totalRadius)
		{
			b3Vec3 normal = -plane2.normal;
			b3Vec3 p1 = 0.5f * (clipEdge1[0].position + r1 * normal + cp1 - r2 * normal);
			b3Vec3 p2 = 0.5f * (clipEdge1[1].position + r1 * normal + cp2 - r2 * normal);

			manifold.pointCount = 2;
			
			manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
			manifold.points[0].key = b3MakeKey(clipEdge1[0].pair);
			manifold.points[0].localNormal = b3MulT(xf1.rotation, normal);
			manifold.points[0].localPoint = b3MulT(xf1, clipEdge1[0].position);
			manifold.points[0].localPoint2 = b3MulT(xf2, cp1);

			manifold.points[1].triangleKey = B3_NULL_TRIANGLE;
			manifold.points[1].key = b3MakeKey(clipEdge1[1].pair);
			manifold.points[1].localNormal = b3MulT(xf1.rotation, normal);
			manifold.points[1].localPoint = b3MulT(xf1, clipEdge1[1].position);
			manifold.points[1].localPoint2 = b3MulT(xf2, cp2);

			manifold.center = 0.5f * (p1 + p2);
			manifold.normal = normal;
			manifold.tangent1 = b3Perp(normal);
			manifold.tangent2 = b3Cross(manifold.tangent1, normal);
		}
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
