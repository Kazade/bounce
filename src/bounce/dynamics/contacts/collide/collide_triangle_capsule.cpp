/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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
#include <bounce/dynamics/shapes/triangle_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/triangle_hull.h>

static void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleHull* hull1, u32 index1, 
	const b3Transform& xf2, const b3Capsule* hull2)
{
	const b3HalfEdge* edge1 = hull1->GetEdge(index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * hull1->centroid;
	b3Vec3 P1 = xf1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	scalar L1 = N1.Normalize();
	B3_ASSERT(L1 > scalar(0));

	b3Vec3 P2 = xf2 * hull2->vertex1;
	b3Vec3 Q2 = xf2 * hull2->vertex2;
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = E2;
	scalar L2 = N2.Normalize();
	B3_ASSERT(L2 > scalar(0));

	// Compute the closest points on the two lines.
	scalar b = b3Dot(N1, N2);
	scalar den = scalar(1) - b * b;
	if (den == scalar(0))
	{
		return;
	}

	scalar inv_den = scalar(1) / den;

	b3Vec3 E3 = P1 - P2;

	scalar d = b3Dot(N1, E3);
	scalar e = b3Dot(N2, E3);

	scalar s = inv_den * (b * e - d);
	scalar t = inv_den * (e - b * d);

	b3Vec3 c1 = P1 + s * N1;
	b3Vec3 c2 = P2 + t * N2;

	// Ensure normal orientation to capsule.
	b3Vec3 N = b3Cross(E1, E2);
	scalar LN = N.Normalize();
	B3_ASSERT(LN > scalar(0));
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	b3FeaturePair pair = b3MakePair(index1, index1 + 1, 0, 1);

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulC(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, c1);
	manifold.points[0].localPoint2 = b3MulT(xf2, c2);
	manifold.points[0].key = b3MakeKey(pair);
}

static void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleHull* hull1, u32 index1, scalar r1,
	const b3Transform& xf2, const b3Capsule* hull2, scalar r2)
{
	b3Capsule worldHull2(xf2 * hull2->vertex1, xf2 * hull2->vertex2, r2);
	b3ClipVertex edge2[2];
	b3BuildEdge(edge2, &worldHull2);

	// Clip edge 2 against the side planes of the reference face.
	b3ClipVertex clipEdge2[2];
	u32 clipCount = b3ClipEdgeToFace(clipEdge2, edge2, xf1, r1, index1, hull1);

	// Project clipped edge 2 onto the reference face.
	b3Plane localPlane1 = hull1->GetPlane(index1);
	b3Plane plane1 = xf1 * localPlane1;

	scalar totalRadius = r1 + r2;

	u32 pointCount = 0;
	for (u32 i = 0; i < clipCount; ++i)
	{
		b3Vec3 c2 = clipEdge2[i].position;
		scalar s = b3Distance(c2, plane1);
		if (s <= totalRadius)
		{
			b3Vec3 c1 = b3ClosestPointOnPlane(c2, plane1);

			b3ManifoldPoint* mp = manifold.points + pointCount;
			mp->localNormal1 = localPlane1.normal;
			mp->localPoint1 = b3MulT(xf1, c1);
			mp->localPoint2 = b3MulT(xf2, c2);
			mp->key = b3MakeKey(clipEdge2[i].pair);

			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

void b3CollideTriangleAndCapsule(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* s1,
	const b3Transform& xf2, const b3CapsuleShape* s2)
{
	scalar r1 = s1->m_radius;
	scalar r2 = s2->m_radius;

	scalar totalRadius = r1 + r2;

	b3ShapeGJKProxy proxy1(s1, 0);
	b3ShapeGJKProxy proxy2(s2, 0);

	b3SimplexCache simplex;
	simplex.count = 0;

	b3GJKOutput gjk = b3GJK(xf1, proxy1, xf2, proxy2, false, &simplex);

	if (gjk.distance > totalRadius)
	{
		return;
	}

	b3TriangleHull hull1(s1->m_vertex1, s1->m_vertex2, s1->m_vertex3);
	b3Capsule hull2(s2->m_vertex1, s2->m_vertex2, r2);

	if (gjk.distance > scalar(0))
	{
		b3Vec3 c1 = gjk.point1;
		b3Vec3 c2 = gjk.point2;
		scalar d = gjk.distance;

		// Define reference normal.
		b3Vec3 N1 = (c2 - c1) / d;

		// Search reference face.
		b3Vec3 localN1 = b3MulC(xf1.rotation, N1);
		u32 index1 = hull1.GetSupportFace(localN1);
		b3Vec3 localFaceN1 = hull1.GetPlane(index1).normal;

		// Paralell vectors |v1xv2| = sin(theta)
		const scalar kTol = scalar(0.005);
		b3Vec3 N = b3Cross(localN1, localFaceN1);
		scalar L = b3Dot(N, N);
		if (L < kTol * kTol)
		{
			// Reference face found.
			// Try to build a face contact.
			b3BuildFaceContact(manifold, xf1, &hull1, index1, r1, xf2, &hull2, r2);
			if (manifold.pointCount == 2)
			{
				return;
			}
		}

		if (s1->m_hasE1Vertex || s1->m_hasE2Vertex || s1->m_hasE3Vertex)
		{
			b3GJKFeaturePair featurePair = b3GetFeaturePair(simplex);

			if (featurePair.count1 == 2)
			{
				u32 v1 = featurePair.index1[0];
				u32 v2 = featurePair.index1[1];

				b3Vec3 vertices[3] = { s1->m_vertex1, s1->m_vertex2, s1->m_vertex3 };
				bool hasWing[3] = { s1->m_hasE1Vertex, s1->m_hasE2Vertex, s1->m_hasE3Vertex };
				b3Vec3 edgeWings[3] = { s1->m_e1Vertex, s1->m_e2Vertex, s1->m_e3Vertex };

				bool edgeFound = false;
				u32 edgeIndex;
				for (u32 i = 0; i < 3; ++i)
				{
					u32 j = i + 1 < 3 ? i + 1 : 0;

					if (v1 == i && v2 == j)
					{
						edgeFound = true;
						edgeIndex = i;
						break;
					}

					if (v2 == i && v1 == j)
					{
						edgeFound = true;
						edgeIndex = i;
						break;
					}
				}

				B3_ASSERT(edgeFound == true);

				if (hasWing[edgeIndex] == true)
				{
					u32 ev1 = edgeIndex;
					u32 ev2 = edgeIndex + 1 < 3 ? edgeIndex + 1 : 0;

					// Put the closest point on the capsule to the frame of the triangle.
					b3Vec3 Q = b3MulT(xf1, c2);

					// Adjacent face triangle
					b3Vec3 A1 = edgeWings[edgeIndex];
					b3Vec3 B1 = vertices[ev2];
					b3Vec3 C1 = vertices[ev1];

					scalar wABC1[4];
					b3BarycentricCoordinates(wABC1, A1, B1, C1, Q);

					// Is the closest point on the capsule in the Region ABC of the adjacent face?
					if (wABC1[0] > scalar(0) && wABC1[1] > scalar(0) && wABC1[2] > scalar(0))
					{
						return;
					}
					
					b3Vec3 center1 = (A1 + B1 + C1) / scalar(3);

					b3Plane frontPlane = hull1.trianglePlanes[0];

					scalar pd = b3Distance(center1, frontPlane);

					const scalar kCoplanarTol = 0.005f;
					
					// Is the edge coplanar?
					if (pd > -kCoplanarTol && pd < kCoplanarTol)
					{
						b3Vec3 n = frontPlane.normal;

						if (b3Dot(n, localN1) < scalar(0))
						{
							n = -n;
						}

						// c1 is constant
						// c2 = c1 + s * n1
						// c1 = c2 - s * n1
						b3Vec3 nc2 = c1 + d * b3Mul(xf1.rotation, n);

						localN1 = n;
						c2 = nc2;
					}
				}
			}
		}

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = localN1;
		manifold.points[0].localPoint1 = b3MulT(xf1, c1);
		manifold.points[0].localPoint2 = b3MulT(xf2, c2);
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, &hull1, xf2, &hull2);
	if (faceQuery1.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, &hull1, xf2, &hull2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	const scalar kTol = scalar(0.1) * B3_LINEAR_SLOP;
	if (edgeQuery.separation > faceQuery1.separation + kTol)
	{
		b3BuildEdgeContact(manifold, xf1, &hull1, edgeQuery.index1, xf2, &hull2);
	}
	else
	{
		b3BuildFaceContact(manifold, xf1, &hull1, faceQuery1.index, r1, xf2, &hull2, r2);
	}
}