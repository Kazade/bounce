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
#include <bounce/dynamics/contacts/contact_cluster.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/hull.h>

void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, u32 index1, const b3HullShape* s1,
	const b3Transform& xf2, u32 index2, const b3HullShape* s2)
{
	const b3Hull* hull1 = s1->m_hull;
	const b3HalfEdge* edge1 = hull1->GetEdge(index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * hull1->centroid;
	b3Vec3 P1 = xf1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	float32 L1 = N1.Normalize();
	B3_ASSERT(L1 > B3_LINEAR_SLOP);

	const b3Hull* hull2 = s2->m_hull;
	const b3HalfEdge* edge2 = hull2->GetEdge(index2);
	const b3HalfEdge* twin2 = hull2->GetEdge(index2 + 1);

	b3Vec3 C2 = xf2 * hull2->centroid;
	b3Vec3 P2 = xf2 * hull2->GetVertex(edge2->origin);
	b3Vec3 Q2 = xf2 * hull2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = E2;
	float32 L2 = N2.Normalize();
	B3_ASSERT(L2 > B3_LINEAR_SLOP);

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
	if (b3Dot(N, P1 - C1) < 0.0f)
	{
		N = -N;
	}

	b3FeaturePair pair = b3MakePair(index1, index1 + 1, index2, index2 + 1);

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulT(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, c1);
	manifold.points[0].localPoint2 = b3MulT(xf2, c2);
	manifold.points[0].key = b3MakeKey(pair);
}

void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, u32 index1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	bool flipNormal)
{
	const b3Hull* hull1 = s1->m_hull;
	float32 r1 = s1->m_radius;

	const b3Hull* hull2 = s2->m_hull;
	float32 r2 = s2->m_radius;

	float32 totalRadius = r1 + r2;

	// 1. Define the reference face plane (1).
	const b3Face* face1 = hull1->GetFace(index1);
	const b3HalfEdge* edge1 = hull1->GetEdge(face1->edge);
	b3Plane localPlane1 = hull1->GetPlane(index1);
	b3Vec3 localNormal1 = localPlane1.normal;
	b3Vec3 localPoint1 = hull1->GetVertex(edge1->origin);
	b3Plane plane1 = b3Mul(xf1, localPlane1);

	// 2. Find the incident face polygon (2).	

	// Put the reference plane normal in the frame of the incident hull (2).
	b3Vec3 normal1 = b3MulT(xf2.rotation, plane1.normal);

	// Find the support face polygon in the *negated* direction.
	b3StackArray<b3ClipVertex, 32> polygon2;
	u32 index2 = hull2->GetSupportFace(-normal1);
	b3BuildPolygon(polygon2, xf2, index2, hull2);

	// 3. Clip incident face polygon (2) against the reference face (1) side planes.
	b3StackArray<b3ClipVertex, 32> clipPolygon2;
	b3ClipPolygonToFace(clipPolygon2, polygon2, xf1, totalRadius, index1, hull1);
	if (clipPolygon2.IsEmpty())
	{
		return;
	}

	// 4. Project the clipped polygon on the reference plane for reduction.
	// Ensure the deepest point is contained in the reduced polygon.
	b3StackArray<b3ClusterPolygonVertex, 32> polygon1;

	u32 minIndex = 0;
	float32 minSeparation = B3_MAX_FLOAT;

	for (u32 i = 0; i < clipPolygon2.Count(); ++i)
	{
		b3ClipVertex v2 = clipPolygon2[i];
		float32 separation = b3Distance(v2.position, plane1);

		if (separation <= totalRadius)
		{
			if (separation < minSeparation)
			{
				minIndex = polygon1.Count();
				minSeparation = separation;
			}

			b3ClusterPolygonVertex v1;
			v1.position = b3ClosestPointOnPlane(v2.position, plane1);
			v1.clipIndex = i;
			polygon1.PushBack(v1);
		}
	}

	if (polygon1.IsEmpty())
	{
		return;
	}

	// 5. Reduce.
	b3Vec3 normal = plane1.normal;

	// Ensure normal orientation to hull 2.
	b3Vec3 s_normal = flipNormal ? -normal : normal;

	b3StackArray<b3ClusterPolygonVertex, 32> reducedPolygon1;
	b3ReducePolygon(reducedPolygon1, polygon1, s_normal, minIndex);
	B3_ASSERT(!reducedPolygon1.IsEmpty());

	// 6. Build face contact.
	u32 pointCount = reducedPolygon1.Count();
	for (u32 i = 0; i < pointCount; ++i)
	{
		u32 clipIndex = reducedPolygon1[i].clipIndex;
		b3ClipVertex v2 = clipPolygon2[clipIndex];
		b3Vec3 v1 = b3ClosestPointOnPlane(v2.position, plane1);

		b3ManifoldPoint* mp = manifold.points + i;

		if (flipNormal)
		{
			// Swap the feature pairs.
			b3FeaturePair pair = b3MakePair(v2.pair.inEdge2, v2.pair.inEdge1, v2.pair.outEdge2, v2.pair.outEdge1);

			mp->localNormal1 = b3MulT(xf2.rotation, s_normal);
			mp->localPoint1 = b3MulT(xf2, v2.position);
			mp->localPoint2 = b3MulT(xf1, v1);
			mp->key = b3MakeKey(pair);
		}
		else
		{
			mp->localNormal1 = b3MulT(xf1.rotation, normal);
			mp->localPoint1 = b3MulT(xf1, v1);
			mp->localPoint2 = b3MulT(xf2, v2.position);
			mp->key = b3MakeKey(v2.pair);
		}
	}

	manifold.pointCount = pointCount;
}

void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2)
{
	B3_ASSERT(manifold.pointCount == 0);

	const b3Hull* hull1 = s1->m_hull;
	float32 r1 = s1->m_radius;

	const b3Hull* hull2 = s2->m_hull;
	float32 r2 = s2->m_radius;

	float32 totalRadius = r1 + r2;

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, hull1, xf2, hull2);
	if (faceQuery1.separation > totalRadius)
	{
		return;
	}

	b3FaceQuery faceQuery2 = b3QueryFaceSeparation(xf2, hull2, xf1, hull1);
	if (faceQuery2.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, hull1, xf2, hull2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	const float32 kTol = 0.1f * B3_LINEAR_SLOP;
	if (edgeQuery.separation > b3Max(faceQuery1.separation, faceQuery2.separation) + kTol)
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, s1, xf2, edgeQuery.index2, s2);
	}
	else
	{
		if (faceQuery1.separation + kTol > faceQuery2.separation)
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, s1, xf2, s2, false);
		}
		else
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, s2, xf1, s1, true);
		}
	}

	// Heuristic succeded. 
	if (manifold.pointCount > 0)
	{
		return;
	}

	// Heuristic failed. Fallback.
	if (edgeQuery.separation > b3Max(faceQuery1.separation, faceQuery2.separation))
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, s1, xf2, edgeQuery.index2, s2);
	}
	else
	{
		if (faceQuery1.separation > faceQuery2.separation)
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, s1, xf2, s2, false);
		}
		else
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, s2, xf1, s1, true);
		}
	}

	// When both convex hulls are not simplified clipping might fail and create no contact points.
	// For example, when a hull contains tiny faces, coplanar faces, and/or non-sharped edges.
	// So we simply create a contact point between the segments.
	// The hulls might overlap, but is better than solving no contact points.
	if (manifold.pointCount == 0)
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, s1, xf2, edgeQuery.index2, s2);
	}

	// If the shapes are overlapping then at least on point must be created.
	B3_ASSERT(manifold.pointCount > 0);
}

bool b3_convexCache = true;
u32 b3_convexCalls = 0, b3_convexCacheHits = 0;

void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	b3FeatureCache* cache);

void b3CollideHullAndHull(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	b3ConvexCache* cache)
{
	++b3_convexCalls;

	if (b3_convexCache)
	{
		b3CollideHulls(manifold, xf1, s1, xf2, s2, &cache->featureCache);
	}
	else
	{
		b3CollideHulls(manifold, xf1, s1, xf2, s2);
	}
}