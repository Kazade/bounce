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
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB, 
	const b3EdgeQuery& query)
{
	u32 indexA = query.indexA;

	const b3HalfEdge* edge1 = hullA->GetEdge(indexA);
	const b3HalfEdge* twin1 = hullA->GetEdge(indexA + 1);

	b3Vec3 C1 = b3Mul(xfA, hullA->centroid);
	b3Vec3 P1 = b3Mul(xfA, hullA->GetVertex(edge1->origin));
	b3Vec3 Q1 = b3Mul(xfA, hullA->GetVertex(twin1->origin));
	b3Vec3 E1 = Q1 - P1;

	u32 indexB = query.indexB;
	
	const b3HalfEdge* edge2 = hullB->GetEdge(indexB);
	const b3HalfEdge* twin2 = hullB->GetEdge(indexB + 1);

	b3Vec3 C2 = b3Mul(xfB, hullB->centroid);
	b3Vec3 P2 = b3Mul(xfB, hullB->GetVertex(edge2->origin));
	b3Vec3 Q2 = b3Mul(xfB, hullB->GetVertex(twin2->origin));
	b3Vec3 E2 = Q2 - P2;

	b3Vec3 N = b3Cross(E1, E2);
	if (b3Dot(N, P1 - C1) < 0.0f)
	{
		N = -N;
	}
	N.Normalize();

	b3Vec3 PA, PB;
	b3ClosestPointsOnLines(&PA, &PB, P1, E1, P2, E2);

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

void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB,
	const b3FaceQuery& query, bool flipNormal)
{
	// 1. Define the reference face plane (A).
	u32 indexA = query.index;
	const b3Face* faceA = hullA->GetFace(indexA);
	const b3HalfEdge* edgeA = hullA->GetEdge(faceA->edge);
	b3Plane localPlaneA = hullA->GetPlane(indexA);
	b3Vec3 localNormalA = localPlaneA.normal;
	b3Vec3 localPointA = hullA->GetVertex(edgeA->origin);
	b3Plane planeA = b3Mul(xfA, localPlaneA);

	// 2. Find the incident face polygon (B).	
	
	// Put the reference plane normal in the frame of the incident hull (B).
	b3Vec3 normalA = b3MulT(xfB.rotation, planeA.normal);

	// Find the support polygon in the *negated* direction.
	b3StackArray<b3ClipVertex, 32> polygonB;
	u32 indexB = hullB->GetSupportFace(-normalA);
	b3BuildPolygon(polygonB, xfB, indexB, hullB);

	// 3. Clip incident face polygon (B) against the reference face (A) side planes.
	b3StackArray<b3ClipVertex, 32> clipPolygonB;
	b3ClipPolygonToFace(clipPolygonB, polygonB, xfA, indexA, hullA);
	if (clipPolygonB.IsEmpty())
	{
		return;
	}

	// 4. Project the clipped polygon on the reference plane for reduction.
	// Ensure the deepest point is contained in the reduced polygon.
	b3StackArray<b3ClusterVertex, 32> polygonA;
	
	u32 minIndex = 0;
	float32 minSeparation = B3_MAX_FLOAT;

	for (u32 i = 0; i < clipPolygonB.Count(); ++i)
	{
		b3ClipVertex vB = clipPolygonB[i];
		float32 separation = b3Distance(vB.position, planeA);

		if (separation <= B3_HULL_RADIUS_SUM)
		{
			if (separation < minSeparation)
			{
				minIndex = polygonA.Count();
				minSeparation = separation;
			}

			b3ClusterVertex vA;
			vA.position = b3Project(vB.position, planeA);
			vA.clipIndex = i;
			polygonA.PushBack(vA);
		}
	}

	if (polygonA.IsEmpty())
	{
		return;
	}

	// 5. Reduce.
	b3StackArray<b3ClusterVertex, 32> reducedPolygonA;
	b3ReducePolygon(reducedPolygonA, polygonA, minIndex);
	B3_ASSERT(!reducedPolygonA.IsEmpty());

	// 6. Build face contact.
	b3Vec3 normal = planeA.normal;
	manifold.center.SetZero();

	u32 pointCount = reducedPolygonA.Count();
	for (u32 i = 0; i < pointCount; ++i)
	{
		u32 clipIndex = reducedPolygonA[i].clipIndex;
		b3ClipVertex vB = clipPolygonB[clipIndex];
		b3Vec3 vA = b3ClosestPointOnPlane(vB.position, planeA);

		b3ManifoldPoint* cp = manifold.points + i;

		if (flipNormal)
		{
			b3FeaturePair pair;
			pair.inEdgeA = vB.pair.inEdgeB;
			pair.outEdgeA = vB.pair.outEdgeB;
			pair.inEdgeB = vB.pair.inEdgeA;
			pair.outEdgeB = vB.pair.outEdgeA;

			cp->triangleKey = B3_NULL_TRIANGLE;
			cp->key = b3MakeKey(pair);
			cp->localNormal = b3MulT(xfB.rotation, -normal);
			cp->localPoint = b3MulT(xfB, vB.position);
			cp->localPoint2 = b3MulT(xfA, vA);
			
			manifold.center += 0.5f * (vA + B3_HULL_RADIUS * normal + vB.position - B3_HULL_RADIUS * normal);
		}
		else
		{
			cp->triangleKey = B3_NULL_TRIANGLE;
			cp->key = b3MakeKey(vB.pair);
			cp->localNormal = b3MulT(xfA.rotation, normal);
			cp->localPoint = b3MulT(xfA, vA);
			cp->localPoint2 = b3MulT(xfB, vB.position);
			
			manifold.center += 0.5f * (vA + B3_HULL_RADIUS * normal + vB.position - B3_HULL_RADIUS * normal);
		}
	}
	
	if (flipNormal)
	{
		localNormalA = -localNormalA;
		normal = -normal;
	}

	manifold.center /= float32(pointCount);
	manifold.normal = normal;
	manifold.tangent1 = b3Perp(normal);
	manifold.tangent2 = b3Cross(manifold.tangent1, normal);
	manifold.pointCount = pointCount;
}

void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	b3FaceQuery faceQueryA = b3QueryFaceSeparation(xfA, hullA, xfB, hullB);
	if (faceQueryA.separation > B3_HULL_RADIUS_SUM)
	{
		return;
	}

	b3FaceQuery faceQueryB = b3QueryFaceSeparation(xfB, hullB, xfA, hullA);
	if (faceQueryB.separation > B3_HULL_RADIUS_SUM)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xfA, hullA, xfB, hullB);
	if (edgeQuery.separation > B3_HULL_RADIUS_SUM)
	{
		return;
	}

	const float32 kRelEdgeTol = 0.90f;
	const float32 kRelFaceTol = 0.98f;
	const float32 kAbsTol = 0.05f;

	if (edgeQuery.separation > kRelEdgeTol * b3Max(faceQueryA.separation, faceQueryB.separation) + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xfA, hullA, xfB, hullB, edgeQuery);
	}
	else
	{
		if (faceQueryA.separation > kRelFaceTol * faceQueryB.separation + kAbsTol)
		{
			b3BuildFaceContact(manifold, xfA, hullA, xfB, hullB, faceQueryA, false);
		}
		else
		{
			b3BuildFaceContact(manifold, xfB, hullB, xfA, hullA, faceQueryB, true);
		}
	}
}

//
bool b3_enableConvexCache = true;
u32 b3_convexCalls = 0, b3_convexCacheHits = 0;

void b3CollideHullAndHull(b3Manifold& manifold,
	const b3Transform& xfA, const b3HullShape* sA,
	const b3Transform& xfB, const b3HullShape* sB,
	b3FeatureCache* cache);

void b3CollideHullAndHull(b3Manifold& manifold,
	const b3Transform& xfA, const b3HullShape* sA,
	const b3Transform& xfB, const b3HullShape* sB, 
	b3ConvexCache* cache)
{
	++b3_convexCalls;
	b3CollideHullAndHull(manifold, xfA, sA, xfB, sB, &cache->featureCache);
}
