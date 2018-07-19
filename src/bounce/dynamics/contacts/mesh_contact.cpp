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

#include <bounce/dynamics/contacts/mesh_contact.h>
#include <bounce/dynamics/contacts/contact_cluster.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/collision/shapes/triangle_hull.h>
#include <bounce/common/memory/stack_allocator.h>

b3MeshContact::b3MeshContact(b3Shape* shapeA, b3Shape* shapeB)
{
	m_type = e_meshContact;

	m_manifoldCapacity = B3_MAX_MANIFOLDS;
	m_manifolds = m_stackManifolds;
	m_manifoldCount = 0;

	b3Transform xfA = shapeA->GetBody()->GetTransform();
	b3Transform xfB = shapeB->GetBody()->GetTransform();

	b3Transform xf = b3MulT(xfB, xfA);
	
	// The fat aabb relative to shape B's frame.
	b3AABB3 fatAABB;
	shapeA->ComputeAABB(&fatAABB, xf);
	fatAABB.Extend(B3_AABB_EXTENSION);

	m_aabbA = fatAABB;
	m_aabbMoved = true;

	// Pre-allocate some indices
	m_triangleCapacity = 16;
	m_triangles = (b3TriangleCache*)b3Alloc(m_triangleCapacity * sizeof(b3TriangleCache));
	m_triangleCount = 0;
}

b3MeshContact::~b3MeshContact()
{
	b3Free(m_triangles);
}

void b3MeshContact::SynchronizeShapes()
{
	b3Shape* shapeA = GetShapeA();
	b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->m_xf;

	b3Shape* shapeB = GetShapeB();
	b3Body* bodyB = shapeB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	b3Sweep* sweepA = &bodyA->m_sweep;
	b3Transform xfA0;
	xfA0.position = sweepA->worldCenter0;
	xfA0.rotation = b3QuatMat33(sweepA->orientation0);
		
	// Calculate the displacement of body A using its position at the last 
	// time step and the current position.
	b3Vec3 displacement = xfA.position - xfA0.position;

	// Compute the AABB in the reference frame of shape B.
	b3Transform xf = b3MulT(xfB, xfA);
	
	b3AABB3 aabb;
	shapeA->ComputeAABB(&aabb, xf);

	// Update the AABB with the new (transformed) AABB and buffer move.
	m_aabbMoved = MoveAABB(aabb, displacement);
}

bool b3MeshContact::MoveAABB(const b3AABB3& aabb, const b3Vec3& displacement)
{
	// Do nothing if the new AABB is contained in the old AABB.
	if (m_aabbA.Contains(aabb))
	{
		// Do nothing if the new AABB is contained in the old AABB.
		return false;
	}

	// Update the AABB with a fat and motion predicted AABB.

	// Extend the new (original) AABB.
	b3AABB3 fatAABB = aabb;
	fatAABB.Extend(B3_AABB_EXTENSION);

	if (displacement.x < 0.0f)
	{
		fatAABB.m_lower.x += B3_AABB_MULTIPLIER * displacement.x;
	}
	else
	{
		fatAABB.m_upper.x += B3_AABB_MULTIPLIER * displacement.x;
	}

	if (displacement.y < 0.0f)
	{
		fatAABB.m_lower.y += B3_AABB_MULTIPLIER * displacement.y;
	}
	else
	{
		fatAABB.m_upper.y += B3_AABB_MULTIPLIER * displacement.y;
	}

	if (displacement.z < 0.0f)
	{
		fatAABB.m_lower.z += B3_AABB_MULTIPLIER * displacement.z;
	}
	else
	{
		fatAABB.m_upper.z += B3_AABB_MULTIPLIER * displacement.z;
	}

	// Update proxy with the extented AABB.
	m_aabbA = fatAABB;

	// Notify the proxy has moved.
	return true;
}

void b3MeshContact::FindNewPairs()
{
	// Reuse the overlapping buffer if the AABB didn't move
	// significantly.
	if (m_aabbMoved == false)
	{
		return;
	}

	// Clear the index cache.
	m_triangleCount = 0;

	const b3MeshShape* meshShapeB = (b3MeshShape*)GetShapeB();
	const b3Mesh* meshB = meshShapeB->m_mesh;
	const b3StaticTree* tree = &meshB->tree;

	// Query and update the overlapping buffer.
	tree->QueryAABB(this, m_aabbA);
}

bool b3MeshContact::Report(u32 proxyId)
{
	b3MeshShape* meshShapeB = (b3MeshShape*)GetShapeB();
	const b3Mesh* meshB = meshShapeB->m_mesh;
	const b3StaticTree* treeB = &meshB->tree;

	u32 triangleIndex = treeB->GetUserData(proxyId);

	// Add the triangle to the overlapping buffer.
	if (m_triangleCount == m_triangleCapacity)
	{
		b3TriangleCache* oldElements = m_triangles;
		m_triangleCapacity *= 2;
		m_triangles = (b3TriangleCache*)b3Alloc(m_triangleCapacity * sizeof(b3TriangleCache));
		memcpy(m_triangles, oldElements, m_triangleCount * sizeof(b3TriangleCache));
		b3Free(oldElements);
	}

	B3_ASSERT(m_triangleCount  < m_triangleCapacity);

	b3TriangleCache* cache = m_triangles + m_triangleCount;
	cache->index = triangleIndex;
	cache->cache.simplexCache.count = 0;
	cache->cache.featureCache.m_featurePair.state = b3SATCacheType::e_empty;
	
	++m_triangleCount;

	// Keep looking for triangles.
	return true;
}

bool b3MeshContact::TestOverlap()
{
	b3Shape* shapeA = GetShapeA();
	b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->GetTransform();
	u32 indexA = 0;

	b3Shape* shapeB = GetShapeB();
	b3Body* bodyB = shapeB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	// Test if at least one triangle of the shape B overlaps the shape A.
	for (u32 i = 0; i < m_triangleCount; ++i)
	{
		b3TriangleCache* cache = m_triangles + i;
		u32 indexB = cache->index;
		bool overlap = b3TestOverlap(xfA, indexA, shapeA, xfB, indexB, shapeB, &cache->cache);
		if (overlap == true)
		{
			return true;
		}
	}

	return false;
}

void b3MeshContact::Collide()
{
	B3_ASSERT(m_manifoldCount == 0);

	b3Shape* shapeA = GetShapeA();
	b3Body* bodyA = shapeA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = GetShapeB();
	b3Body* bodyB = shapeB->GetBody();
	b3MeshShape* meshShapeB = (b3MeshShape*)shapeB;
	b3Transform xfB = bodyB->GetTransform();

	b3World* world = bodyA->GetWorld();
	b3StackAllocator* allocator = &world->m_stackAllocator;

	// Create one manifold per triangle.
	b3Manifold* tempManifolds = (b3Manifold*)allocator->Allocate(m_triangleCount * sizeof(b3Manifold));
	u32 tempCount = 0;

	const b3Mesh* meshB = meshShapeB->m_mesh;
	for (u32 i = 0; i < m_triangleCount; ++i)
	{
		b3TriangleCache* triangleCache = m_triangles + i;
		u32 triangleIndex = triangleCache->index;
		b3Triangle* triangle = meshB->triangles + triangleIndex;

		b3Vec3 v1 = meshB->vertices[triangle->v1];
		b3Vec3 v2 = meshB->vertices[triangle->v2];
		b3Vec3 v3 = meshB->vertices[triangle->v3];

		b3TriangleHull hullB(v1, v2, v3);

		b3HullShape hullShapeB;
		hullShapeB.m_body = bodyB;
		hullShapeB.m_hull = &hullB;
		hullShapeB.m_radius = B3_HULL_RADIUS;
				
		b3Manifold* manifold = tempManifolds + tempCount;
		manifold->Initialize();
		
		b3CollideShapeAndShape(*manifold, xfA, shapeA, xfB, &hullShapeB, &triangleCache->cache);
		
		for (u32 j = 0; j < manifold->pointCount; ++j)
		{
			manifold->points[j].key.triangleKey = triangleIndex;
		}
		
		++tempCount;
	}

	// Send contact manifolds for clustering. This is an important optimization.
	B3_ASSERT(m_manifoldCount == 0);
	
	b3ClusterSolver clusterSolver;
	clusterSolver.Run(m_stackManifolds, m_manifoldCount, tempManifolds, tempCount, xfA, shapeA->m_radius, xfB, B3_HULL_RADIUS);
	
	allocator->Free(tempManifolds);
}