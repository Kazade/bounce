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
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/collision/shapes/hull.h>

void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, u32 index1, const b3Hull* hull1,
	const b3Transform& xf2, u32 index2, const b3Hull* hull2);

void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, float32 r1, u32 index1, const b3Hull* hull1,
	const b3Transform& xf2, float32 r2, const b3Hull* hull2,
	bool flipNormal);

static void b3RebuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, u32 index1, const b3HullShape* s1,
	const b3Transform& xf2, u32 index2, const b3HullShape* s2)
{
	const b3Hull* hull1 = s1->m_hull;
	const b3Hull* hull2 = s2->m_hull;
	
	const b3HalfEdge* edge1 = hull1->GetEdge(index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * hull1->centroid;
	b3Vec3 P1 = xf1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	float32 L1 = N1.Normalize();
	B3_ASSERT(L1 > 0.0f);

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

	// Check if the closest points are still lying on the opposite segments 
	// using Barycentric coordinates.
	float32 w2[3];
	b3Barycentric(w2, P1, Q1, c2);

	float32 w1[3];
	b3Barycentric(w1, P2, Q2, c1);

	if (w2[1] > 0.0f && w2[1] <= w2[2] &&
		w1[1] > 0.0f && w1[1] <= w1[2])
	{
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
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = b3MakeKey(pair);
	}
}

static void b3RebuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, u32 index1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2, bool flipNormal)
{
	const b3Hull* hull1 = s1->m_hull;
	float32 r1 = s1->m_radius;
	const b3Body* body1 = s1->GetBody();

	const b3Hull* hull2 = s2->m_hull;
	float32 r2 = s2->m_radius;
	const b3Body* body2 = s2->GetBody();

	const b3Sweep& sweep1 = body1->GetSweep();
	b3Quat q10 = sweep1.orientation0;
	b3Quat q1 = sweep1.orientation;

	const b3Sweep& sweep2 = body2->GetSweep();
	b3Quat q20 = sweep2.orientation0;
	b3Quat q2 = sweep2.orientation;

	// Check if the relative orientation has changed.
	// Here the second orientation seen by the first orientation.
	// dp = p2 - p1
	// dq * q1 = q2
	// dq = inv(q1) * q2

	// The old relative rotation.
	// "q0(2) - q0(1)"
	b3Quat dq0 = b3Conjugate(q10) * q20;

	// The new relative rotation.
	// "q(2) - q(1)"
	b3Quat dq = b3Conjugate(q1) * q2;
	
	// Relative rotation between the new relative rotation and the old relative rotation.
	// "dq(2) - dq0(1)"
	b3Quat q = b3Conjugate(dq0) * dq;

	// Check the relative absolute cosine because 
	// we want to check orientation similarity.
	const float32 kTol = 0.995f;
	if (b3Abs(q.w) > kTol)
	{
		b3BuildFaceContact(manifold, xf1, r1, index1, hull1, xf2, r2, hull2, flipNormal);
	}
}

void b3CollideCache(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	b3FeatureCache* cache)
{
	B3_ASSERT(cache->m_featurePair.state == b3SATCacheType::e_empty);

	const b3Hull* hull1 = s1->m_hull;
	const b3Hull* hull2 = s2->m_hull;

	float32 r1 = s1->m_radius;
	float32 r2 = s2->m_radius;
	float32 totalRadius = r1 + r2;

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, hull1, xf2, hull2);
	if (faceQuery1.separation > totalRadius)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_face1;
		cache->m_featurePair.index1 = faceQuery1.index;
		cache->m_featurePair.index2 = faceQuery1.index;
		return;
	}

	b3FaceQuery faceQuery2 = b3QueryFaceSeparation(xf2, hull2, xf1, hull1);
	if (faceQuery2.separation > totalRadius)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_face2;
		cache->m_featurePair.index1 = faceQuery2.index;
		cache->m_featurePair.index2 = faceQuery2.index;
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, hull1, xf2, hull2);
	if (edgeQuery.separation > totalRadius)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_edge1;
		cache->m_featurePair.index1 = edgeQuery.index1;
		cache->m_featurePair.index2 = edgeQuery.index2;
		return;
	}

	const float32 kTol = 0.1f * B3_LINEAR_SLOP;

	if (edgeQuery.separation > b3Max(faceQuery1.separation, faceQuery2.separation) + kTol)
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, hull1, xf2, edgeQuery.index2, hull2);
		if(manifold.pointCount > 0)
		{	
			// Write an overlap cache.
			cache->m_featurePair.state = b3SATCacheType::e_overlap;
			cache->m_featurePair.type = b3SATFeaturePair::e_edge1;
			cache->m_featurePair.index1 = edgeQuery.index1;
			cache->m_featurePair.index2 = edgeQuery.index2;		
		}
	}
	else
	{
		if (faceQuery1.separation + kTol > faceQuery2.separation)
		{
			b3BuildFaceContact(manifold, xf1, r1, faceQuery1.index, hull1, xf2, r2, hull2, false);
			if(manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache->m_featurePair.state = b3SATCacheType::e_overlap;
				cache->m_featurePair.type = b3SATFeaturePair::e_face1;
				cache->m_featurePair.index1 = faceQuery1.index;
				cache->m_featurePair.index2 = faceQuery1.index;
			}
		}
		else
		{
			b3BuildFaceContact(manifold, xf2, r2, faceQuery2.index, hull2, xf1, r1, hull1, true);
			if(manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache->m_featurePair.state = b3SATCacheType::e_overlap;
				cache->m_featurePair.type = b3SATFeaturePair::e_face2;
				cache->m_featurePair.index1 = faceQuery2.index;
				cache->m_featurePair.index2 = faceQuery2.index;
			}
		}
	}
}

b3SATCacheType b3FeatureCache::ReadState(
	const b3Transform& xf1, float32 r1, const b3Hull* hull1,
	const b3Transform& xf2, float32 r2, const b3Hull* hull2)
{
	// If the cache was empty or flushed choose an arbitrary feature pair.
	if (m_featurePair.state == b3SATCacheType::e_empty)
	{
		m_featurePair.state = b3SATCacheType::e_separation;
		m_featurePair.type = b3SATFeaturePair::e_face1;
		m_featurePair.index1 = 0;
		m_featurePair.index2 = 0;
	}

	switch (m_featurePair.type)
	{
	case b3SATFeaturePair::e_edge1:
	{
		return ReadEdge(xf1, r1, hull1, xf2, r2, hull2);
	}
	case b3SATFeaturePair::e_face1:
	{
		return ReadFace(xf1, r1, hull1, xf2, r2, hull2);
	}
	case b3SATFeaturePair::e_face2:
	{
		return ReadFace(xf2, r2, hull2, xf1, r1, hull1);
	}
	default:
	{
		return b3SATCacheType::e_empty;
	}
	}
}

b3SATCacheType b3FeatureCache::ReadFace(
	const b3Transform& xf1, float32 r1, const b3Hull* hull1,
	const b3Transform& xf2, float32 r2, const b3Hull* hull2)
{
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);
	b3Plane plane = xf * hull1->GetPlane(m_featurePair.index1);
	float32 separation = b3Project(hull2, plane);
	if (separation > r1 + r2)
	{
		return e_separation;
	}
	return e_overlap;
}

b3SATCacheType b3FeatureCache::ReadEdge(
	const b3Transform& xf1, float32 r1, const b3Hull* hull1,
	const b3Transform& xf2, float32 r2, const b3Hull* hull2)
{
	u32 i = m_featurePair.index1;
	u32 j = m_featurePair.index2;

	// Query minimum separation distance and axis of the first hull planes.
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);
	b3Vec3 C1 = xf * hull1->centroid;

	const b3HalfEdge* edge1 = hull1->GetEdge(i);
	const b3HalfEdge* twin1 = hull1->GetEdge(i + 1);

	B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

	b3Vec3 P1 = xf * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;

	// The Gauss Map of edge 1.
	b3Vec3 U1 = xf.rotation * hull1->GetPlane(edge1->face).normal;
	b3Vec3 V1 = xf.rotation * hull1->GetPlane(twin1->face).normal;

	const b3HalfEdge* edge2 = hull2->GetEdge(j);
	const b3HalfEdge* twin2 = hull2->GetEdge(j + 1);

	B3_ASSERT(edge2->twin == j + 1 && twin2->twin == j);

	b3Vec3 P2 = hull2->GetVertex(edge2->origin);
	b3Vec3 Q2 = hull2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;

	// The Gauss Map of edge 2.
	b3Vec3 U2 = hull2->GetPlane(edge2->face).normal;
	b3Vec3 V2 = hull2->GetPlane(twin2->face).normal;

	// Negate the Gauss Map 2 for account for the MD.
	if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2))
	{
		float32 separation = b3Project(P1, E1, P2, E2, C1);
		if (separation > r1 + r2)
		{
			return b3SATCacheType::e_separation;
		}
		else
		{
			return b3SATCacheType::e_overlap;
		}
	}

	// We can't determine the cache type 
	// therefore must run SAT.
	return b3SATCacheType::e_empty;
}

extern u32 b3_convexCacheHits;

void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	b3FeatureCache* cache)
{
	const b3Hull* hull1 = s1->m_hull;
	float32 r1 = s1->m_radius;
	
	const b3Hull* hull2 = s2->m_hull;
	float32 r2 = s2->m_radius;
	
	// Read cache
	b3SATCacheType state0 = cache->m_featurePair.state;
	b3SATCacheType state1 = cache->ReadState(xf1, r1, hull1, xf2, r2, hull2);
	
	if (state0 == b3SATCacheType::e_separation &&
		state1 == b3SATCacheType::e_separation)
	{
		// Separation cache hit.
		++b3_convexCacheHits;
		return;
	}
	else if (state0 == b3SATCacheType::e_overlap &&
			 state1 == b3SATCacheType::e_overlap)
	{
		// Try to rebuild or reclip the features.
		switch (cache->m_featurePair.type)
		{
		case b3SATFeaturePair::e_edge1:
		{
			b3RebuildEdgeContact(manifold, xf1, cache->m_featurePair.index1, s1, xf2, cache->m_featurePair.index2, s2);
			break;
		}
		case b3SATFeaturePair::e_face1:
		{
			b3RebuildFaceContact(manifold, xf1, cache->m_featurePair.index1, s1, xf2, s2, false);
			break;
		}
		case b3SATFeaturePair::e_face2:
		{
			b3RebuildFaceContact(manifold, xf2, cache->m_featurePair.index1, s2, xf1, s1, true);
			break;
		}
		default:
		{
			break;
		}
		}

		if (manifold.pointCount > 0)
		{
			// Overlap cache hit.
			++b3_convexCacheHits;
			return;
		}
	}

	// Separation cache miss.
	// Overlap cache miss.
	// Flush the cache.
	cache->m_featurePair.state = b3SATCacheType::e_empty;
	b3CollideCache(manifold, xf1, s1, xf2, s2, cache);
}