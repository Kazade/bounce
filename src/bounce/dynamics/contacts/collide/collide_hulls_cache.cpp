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
#include <bounce/collision/shapes/hull.h>

extern u32 b3_convexCacheHits;

void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB,
	const b3EdgeQuery& query);

void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB,
	const b3FaceQuery& query, bool flipNormal);

void b3RebuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xfA, u32 indexA, const b3HullShape* sA,
	const b3Transform& xfB, u32 indexB, const b3HullShape* sB)
{
	const b3Hull* hullA = sA->m_hull;
	const b3Hull* hullB = sB->m_hull;

	const b3HalfEdge* edge1 = hullA->GetEdge(indexA);
	const b3HalfEdge* twin1 = hullA->GetEdge(indexA + 1);

	b3Vec3 C1 = b3Mul(xfA, hullA->centroid);
	b3Vec3 P1 = b3Mul(xfA, hullA->GetVertex(edge1->origin));
	b3Vec3 Q1 = b3Mul(xfA, hullA->GetVertex(twin1->origin));
	b3Vec3 E1 = Q1 - P1;

	const b3HalfEdge* edge2 = hullB->GetEdge(indexB);
	const b3HalfEdge* twin2 = hullB->GetEdge(indexB + 1);

	b3Vec3 C2 = b3Mul(xfB, hullB->centroid);
	b3Vec3 P2 = b3Mul(xfB, hullB->GetVertex(edge2->origin));
	b3Vec3 Q2 = b3Mul(xfB, hullB->GetVertex(twin2->origin));
	b3Vec3 E2 = Q2 - P2;

	b3Vec3 PA, PB;
	b3ClosestPointsOnLines(&PA, &PB, P1, E1, P2, E2);

	// Check if the closest points are still lying on the opposite segments 
	// using Barycentric coordinates.
	float32 wB[3];
	b3Barycentric(wB, P1, Q1, PB);

	float32 wA[3];
	b3Barycentric(wA, P2, Q2, PA);

	if (wB[1] > 0.0f && wB[1] <= wB[2] &&
		wA[1] > 0.0f && wA[1] <= wA[2])
	{
		b3Vec3 N = b3Cross(E1, E2);
		if (b3Dot(N, P1 - C1) < 0.0f)
		{
			N = -N;
		}
		N.Normalize();

		b3FeaturePair pair = b3MakePair(indexA, indexA + 1, indexB, indexB + 1);

		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = b3MakeKey(pair);
		manifold.points[0].localNormal = b3MulT(xfA.rotation, N);
		manifold.points[0].localPoint = b3MulT(xfA, PA);
		manifold.points[0].localPoint2 = b3MulT(xfB, PB);
		
		manifold.center = 0.5f * (PA + B3_HULL_RADIUS * N + PB - B3_HULL_RADIUS * N);
		manifold.normal = N;
		manifold.tangent1 = b3Perp(N);
		manifold.tangent2 = b3Cross(manifold.tangent1, N);
	}
}

void b3RebuildFaceContact(b3Manifold& manifold,
	const b3Transform& xfA, u32 indexA, const b3HullShape* sA,
	const b3Transform& xfB, const b3HullShape* sB, bool flipNormal)
{
	const b3Hull* hullA = sA->m_hull;
	const b3Hull* hullB = sB->m_hull;

	b3FaceQuery query;
	query.index = indexA;
	
	b3BuildFaceContact(manifold, xfA, hullA, xfB, hullB, query, flipNormal);
}

void b3CollideCache(b3Manifold& manifold,
	const b3Transform& xfA, const b3HullShape* sA,
	const b3Transform& xfB, const b3HullShape* sB,
	b3FeatureCache* cache)
{
	B3_ASSERT(cache->m_featurePair.state == b3SATCacheType::e_empty);

	const b3Hull* hullA = sA->m_hull;
	const b3Hull* hullB = sB->m_hull;

	b3FaceQuery faceQueryA = b3QueryFaceSeparation(xfA, hullA, xfB, hullB);
	if (faceQueryA.separation > B3_HULL_RADIUS_SUM)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_faceA;
		cache->m_featurePair.indexA = faceQueryA.index;
		cache->m_featurePair.indexB = faceQueryA.index;
		return;
	}

	b3FaceQuery faceQueryB = b3QueryFaceSeparation(xfB, hullB, xfA, hullA);
	if (faceQueryB.separation > B3_HULL_RADIUS_SUM)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_faceB;
		cache->m_featurePair.indexA = faceQueryB.index;
		cache->m_featurePair.indexB = faceQueryB.index;
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xfA, hullA, xfB, hullB);
	if (edgeQuery.separation > B3_HULL_RADIUS_SUM)
	{
		// Write a separation cache.
		cache->m_featurePair.state = b3SATCacheType::e_separation;
		cache->m_featurePair.type = b3SATFeaturePair::e_edgeA;
		cache->m_featurePair.indexA = edgeQuery.indexA;
		cache->m_featurePair.indexB = edgeQuery.indexB;
		return;
	}

	const float32 kRelEdgeTol = 0.90f;
	const float32 kRelFaceTol = 0.98f;
	const float32 kAbsTol = 0.05f;

	if (edgeQuery.separation > kRelEdgeTol * b3Max(faceQueryA.separation, faceQueryB.separation) + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xfA, hullA, xfB, hullB, edgeQuery);
		if(manifold.pointCount > 0)
		{	
			// Write an overlap cache.
			cache->m_featurePair.state = b3SATCacheType::e_overlap;
			cache->m_featurePair.type = b3SATFeaturePair::e_edgeA;
			cache->m_featurePair.indexA = edgeQuery.indexA;
			cache->m_featurePair.indexB = edgeQuery.indexB;		
		}
	}
	else
	{
		if (faceQueryA.separation > kRelFaceTol * faceQueryB.separation + kAbsTol)
		{
			b3BuildFaceContact(manifold, xfA, hullA, xfB, hullB, faceQueryA, false);
			if(manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache->m_featurePair.state = b3SATCacheType::e_overlap;
				cache->m_featurePair.type = b3SATFeaturePair::e_faceA;
				cache->m_featurePair.indexA = faceQueryA.index;
				cache->m_featurePair.indexB = faceQueryA.index;
			}
		}
		else
		{
			b3BuildFaceContact(manifold, xfB, hullB, xfA, hullA, faceQueryB, true);
			if(manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache->m_featurePair.state = b3SATCacheType::e_overlap;
				cache->m_featurePair.type = b3SATFeaturePair::e_faceB;
				cache->m_featurePair.indexA = faceQueryB.index;
				cache->m_featurePair.indexB = faceQueryB.index;
			}
		}
	}
}

b3SATCacheType b3FeatureCache::ReadState(
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	// If the cache was empty or flushed choose an arbitrary feature pair.
	if (m_featurePair.state == b3SATCacheType::e_empty)
	{
		m_featurePair.state = b3SATCacheType::e_separation;
		m_featurePair.type = b3SATFeaturePair::e_faceA;
		m_featurePair.indexA = 0;
		m_featurePair.indexB = 0;
	}

	switch (m_featurePair.type)
	{
	case b3SATFeaturePair::e_edgeA:
	{
		return ReadEdge(xfA, hullA, xfB, hullB);
	}
	case b3SATFeaturePair::e_faceA:
	{
		return ReadFace(xfA, hullA, xfB, hullB);
	}
	case b3SATFeaturePair::e_faceB:
	{
		return ReadFace(xfB, hullB, xfA, hullA);
	}
	default:
	{
		return b3SATCacheType::e_empty;
	}
	}
}

b3SATCacheType b3FeatureCache::ReadFace(
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xfB, xfA);
	b3Plane plane = xf * hullA->GetPlane(m_featurePair.indexA);
	float32 separation = b3Project(hullB, plane);
	if (separation > B3_HULL_RADIUS_SUM)
	{
		return e_separation;
	}
	return e_overlap;
}

b3SATCacheType b3FeatureCache::ReadEdge(
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	u32 i = m_featurePair.indexA;
	u32 j = m_featurePair.indexB;

	// Query minimum separation distance and axis of the first hull planes.
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xfB, xfA);
	b3Vec3 C1 = xf * hullA->centroid;

	const b3HalfEdge* edge1 = hullA->GetEdge(i);
	const b3HalfEdge* twin1 = hullA->GetEdge(i + 1);

	B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

	b3Vec3 P1 = xf * hullA->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf * hullA->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;

	// The Gauss Map of edge 1.
	b3Vec3 U1 = xf.rotation * hullA->GetPlane(edge1->face).normal;
	b3Vec3 V1 = xf.rotation * hullA->GetPlane(twin1->face).normal;

	const b3HalfEdge* edge2 = hullB->GetEdge(j);
	const b3HalfEdge* twin2 = hullB->GetEdge(j + 1);

	B3_ASSERT(edge2->twin == j + 1 && twin2->twin == j);

	b3Vec3 P2 = hullB->GetVertex(edge2->origin);
	b3Vec3 Q2 = hullB->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;

	// The Gauss Map of edge 2.
	b3Vec3 U2 = hullB->GetPlane(edge2->face).normal;
	b3Vec3 V2 = hullB->GetPlane(twin2->face).normal;

	// Negate the Gauss Map 2 for account for the MD.
	if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2))
	{
		float32 separation = b3Project(P1, E1, P2, E2, C1);
		if (separation > B3_HULL_RADIUS_SUM)
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

void b3CollideHullAndHull(b3Manifold& manifold,
	const b3Transform& xfA, const b3HullShape* sA,
	const b3Transform& xfB, const b3HullShape* sB,
	b3FeatureCache* cache)
{
	const b3Hull* hullA = sA->m_hull;
	const b3Hull* hullB = sB->m_hull;

	// Read cache
	b3SATCacheType state0 = cache->m_featurePair.state;
	b3SATCacheType state1 = cache->ReadState(xfA, hullA, xfB, hullB);
	
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
		case b3SATFeaturePair::e_edgeA:
		{
			b3RebuildEdgeContact(manifold, xfA, cache->m_featurePair.indexA, sA, xfB, cache->m_featurePair.indexB, sB);
			break;
		}
		case b3SATFeaturePair::e_faceA:
		{
			b3RebuildFaceContact(manifold, xfA, cache->m_featurePair.indexA, sA, xfB, sB, false);
			break;
		}
		case b3SATFeaturePair::e_faceB:
		{
			b3RebuildFaceContact(manifold, xfB, cache->m_featurePair.indexA, sB, xfA, sA, true);
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
	b3CollideCache(manifold, xfA, sA, xfB, sB, cache);
}
