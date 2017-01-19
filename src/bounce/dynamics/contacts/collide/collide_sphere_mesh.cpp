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
* 3. This notice may not be RemoveTriangled or altered from any source distribution.
*/

#include <bounce/dynamics/contacts/mesh_contact.h>
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/dynamics/contacts/contact_cluster.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/collision/gjk/gjk_cache.h>
#include <bounce/common/memory/stack_allocator.h>

// Implementation based on Peter Tchernev's relevant quote 
// in the Bullet forum.
// The mesh must contain unique vertices for this collision
// filtering algorithm to work correctly.
// Otherwise we'll need to switch to comparisons 
// using the binary representation of each vertex coordinate, 
// which will be slight slower.
// Therefore it is recommended for your resource compiler 
// to enforce this restriction on each physics mesh.

struct b3FeaturePoint
{
	u32 index; 
	b3GJKOutput output;
	b3GJKFeaturePair pair;
};

// This is used for fast sorting.
struct b3SortKey
{
	bool operator<(const b3SortKey& b) const
	{
		return distance < b.distance;
	}

	u32 index; // original index to contact point
	float32 distance; // distance between closest points
};

struct b3SMCollider
{
	b3SMCollider(b3StackAllocator* allocator,
		b3TriangleCache* triangles, u32 triangleCount,
		const b3Transform& xfA, const b3SphereShape* sA,
		const b3Transform& xfB, const b3MeshShape* sB);

	~b3SMCollider();

	u32 Collide(b3Manifold manifolds[3]);
	void CollideTriangle(u32 i);
	void Filter();

	bool ShouldCollide(u32 v) const;
	bool ShouldCollide(u32 v1, u32 v2) const;
	bool ShouldCollide(const b3FeaturePoint* p) const;

	void AddPoint(u32 triangle, const b3GJKOutput& output, const b3GJKFeaturePair& pair);
	void AddDelayedPoint(u32 triangle, const b3GJKOutput& output, const b3GJKFeaturePair& pair);
	void AddValidPoint(u32 triangle, const b3GJKOutput& output, const b3GJKFeaturePair& pair);
	void RemoveTriangle(const b3Triangle* triangle);

	b3StackAllocator* m_alloc;

	b3ShapeGJKProxy m_proxyA;
	b3Vec3 m_localVertexA;
	b3Vec3 m_vertexA;
	float32 m_rA;
	b3Transform m_xfA;

	b3ShapeGJKProxy m_proxyB;
	const b3MeshShape* m_meshB;
	float32 m_rB;
	b3Transform m_xfB;

	float32 m_totalRadius;

	b3TriangleCache* m_triangles;
	u32 m_triangleCount;

	b3Manifold* m_manifolds;
	u32 m_manifoldCount;

	// Triangles that cannot collide anymore.
	b3Triangle* m_removeds;
	u32 m_removedCount;

	// Vertex or edge contact points that can collide.
	b3FeaturePoint* m_delayeds;
	b3SortKey* m_keys;
	u32 m_delayedCount;
};

b3SMCollider::b3SMCollider(b3StackAllocator* allocator,
	b3TriangleCache* triangles, u32 triangleCount,
	const b3Transform& xfA, const b3SphereShape* sA,
	const b3Transform& xfB, const b3MeshShape* sB)
{
	m_alloc = allocator;
	m_triangles = triangles;
	m_triangleCount = triangleCount;

	m_xfA = xfA;
	m_xfB = xfB;

	m_proxyA.Set(sA, 0);
	m_localVertexA = sA->m_center;
	m_vertexA = b3Mul(m_xfA, m_localVertexA);
	m_rA = sA->m_radius;

	m_meshB = sB;
	m_rB = sB->m_radius;

	m_totalRadius = m_rA + m_rB;

	m_delayeds = (b3FeaturePoint*)m_alloc->Allocate(m_triangleCount * sizeof(b3FeaturePoint));
	m_keys = (b3SortKey*)m_alloc->Allocate(m_triangleCount * sizeof(b3SortKey));
	m_delayedCount = 0;

	m_removeds = (b3Triangle*)m_alloc->Allocate(m_triangleCount * sizeof(b3Triangle));
	m_removedCount = 0;

	m_manifolds = (b3Manifold*)m_alloc->Allocate(m_triangleCount * sizeof(b3Manifold));
	m_manifoldCount = 0;
}

b3SMCollider::~b3SMCollider()
{
	m_alloc->Free(m_manifolds);
	m_alloc->Free(m_removeds);
	m_alloc->Free(m_keys);
	m_alloc->Free(m_delayeds);
}

u32 b3SMCollider::Collide(b3Manifold manifolds[3])
{
	for (u32 i = 0; i < m_triangleCount; ++i)
	{
		CollideTriangle(i);
	}

	Filter();

	u32 numOut = b3Clusterize(manifolds, m_manifolds, m_manifoldCount, m_xfA, m_rA, m_xfB, m_rB);
	return numOut;
}

void b3SMCollider::CollideTriangle(u32 i)
{
	b3TriangleCache* t = m_triangles + i;
	
	// GJK
	b3ShapeGJKProxy proxyB(m_meshB, t->index);
	b3GJKOutput output = b3GJK(m_xfA, m_proxyA, m_xfB, proxyB, false, &t->cache.simplexCache);

	if (output.distance > m_totalRadius)
	{
		return;
	}

	if (output.distance > B3_EPSILON)
	{
		b3GJKFeaturePair pair = b3GetFeaturePair(t->cache.simplexCache);
		AddPoint(i, output, pair);
		return;
	}

	// SAT
	b3Triangle* triangle = m_meshB->m_mesh->triangles + t->index;
	b3Vec3 A = b3Mul(m_xfB, m_meshB->m_mesh->vertices[triangle->v1]);
	b3Vec3 B = b3Mul(m_xfB, m_meshB->m_mesh->vertices[triangle->v2]);
	b3Vec3 C = b3Mul(m_xfB, m_meshB->m_mesh->vertices[triangle->v3]);

	b3Plane plane(A, B, C);
	float32 separation = b3Distance(m_vertexA, plane);
	float32 sign = b3Sign(separation);

	if (sign * separation > m_totalRadius)
	{
		return;
	}

	b3Vec3 normal = sign * plane.normal;

	B3_ASSERT(m_manifoldCount < m_triangleCount);
	b3Manifold* m = m_manifolds + m_manifoldCount;
	m->GuessImpulses();
	++m_manifoldCount;
	
	b3Vec3 p1 = m_vertexA;
	b3Vec3 p2 = b3ClosestPointOnPlane(p1, plane);

	// Ensure normal orientation to shape B.
	normal = -normal;

	m->pointCount = 1;
	m->points[0].triangleKey = t->index;
	m->points[0].key = 0;
	m->points[0].localNormal = b3MulT(m_xfA.rotation, normal);
	m->points[0].localPoint = m_localVertexA;
	m->points[0].localPoint2 = p2;

	m->center = 0.5f * (p1 + m_rA * normal + p2 - m_rB * normal);
	m->normal = normal;
	m->tangent1 = b3Perp(normal);
	m->tangent2 = b3Cross(m->tangent1, normal);

	RemoveTriangle(triangle);
}

void b3SMCollider::Filter()
{
	// Sort contact points according to distance to triangles.
	std::sort(m_keys, m_keys + m_delayedCount);

	for (u32 i = 0; i < m_delayedCount; ++i)
	{
		const b3SortKey* key = m_keys + i;
		const b3FeaturePoint* p = m_delayeds + key->index;
		const b3TriangleCache* tc = m_triangles + p->index;
		const b3Triangle* t = m_meshB->m_mesh->triangles + tc->index;

		bool ok = ShouldCollide(p);
		if (ok)
		{
			AddValidPoint(p->index, p->output, p->pair);
		}

		// Now this triangle cannot collide anymore.
		RemoveTriangle(t);
	}
}

void b3SMCollider::RemoveTriangle(const b3Triangle* triangle)
{
	B3_ASSERT(m_removedCount < m_triangleCount);
	m_removeds[m_removedCount] = *triangle;
	++m_removedCount;
}

bool b3SMCollider::ShouldCollide(u32 v) const
{
	for (u32 i = 0; i < m_removedCount; ++i)
	{
		if (m_removeds[i].TestVertex(v))
		{
			return false;
		}
	}
	return true;
}

bool b3SMCollider::ShouldCollide(u32 v1, u32 v2) const
{
	for (u32 i = 0; i < m_removedCount; ++i)
	{
		if (m_removeds[i].TestEdge(v1, v2))
		{
			return false;
		}
	}
	
	return true;
}

bool b3SMCollider::ShouldCollide(const b3FeaturePoint* p) const
{
	b3TriangleCache* tc = m_triangles + p->index;
	b3Triangle* t = m_meshB->m_mesh->triangles + tc->index;
	u32 is[3] = { t->v1, t->v2, t->v3 };

	switch (p->pair.countB)
	{
	case 1:
	{
		u32 v1 = is[p->pair.indexB[0]];
		return ShouldCollide(v1);
	}
	case 2:
	{
		u32 v1 = is[p->pair.indexB[0]];
		u32 v2 = is[p->pair.indexB[1]];
		return ShouldCollide(v1, v2);
	}
	case 3:
	{
		return true;
	}
	default:
	{
		B3_ASSERT(false);
		return true;
	}
	}

	return false;
}

void b3SMCollider::AddPoint(u32 i, const b3GJKOutput& output, const b3GJKFeaturePair& pair)
{
	if (pair.countB == 3)
	{
		b3TriangleCache* tc = m_triangles + i;
		b3Triangle* t = m_meshB->m_mesh->triangles + tc->index;
		
		AddValidPoint(i, output, pair);
		RemoveTriangle(t);
	}
	else
	{
		AddDelayedPoint(i, output, pair);
	}
}

void b3SMCollider::AddDelayedPoint(u32 i, const b3GJKOutput& output, const b3GJKFeaturePair& pair)
{
	b3TriangleCache* tc = m_triangles + i;
	b3Triangle* t = m_meshB->m_mesh->triangles + tc->index;

	B3_ASSERT(m_delayedCount < m_triangleCount);
	b3FeaturePoint* p = m_delayeds + m_delayedCount;
	b3SortKey* k = m_keys + m_delayedCount;

	p->index = i;
	p->output = output;
	p->pair = pair;
	k->index = m_delayedCount;
	k->distance = output.distance;

	++m_delayedCount;
}

void b3SMCollider::AddValidPoint(u32 i, const b3GJKOutput& output, const b3GJKFeaturePair& pair)
{
	b3TriangleCache* tc = m_triangles + i;
	b3Triangle* t = m_meshB->m_mesh->triangles + tc->index;
	
	B3_ASSERT(m_manifoldCount < m_triangleCount);
	b3Manifold* m = m_manifolds + m_manifoldCount;
	++m_manifoldCount;
	
	m->GuessImpulses();
	
	b3Vec3 p1 = output.pointA;
	b3Vec3 p2 = output.pointB;
	b3Vec3 n = (p2 - p1) / output.distance;

	m->pointCount = 1;
	m->points[0].triangleKey = tc->index;
	m->points[0].key = 0;
	m->points[0].localNormal = b3MulT(m_xfA.rotation, n);
	m->points[0].localPoint = m_localVertexA;
	m->points[0].localPoint2 = b3MulT(m_xfB, p2);

	m->center = 0.5f * (p1 + m_rA * n + p2 - m_rB * n);
	m->normal = n;
	m->tangent1 = b3Perp(n);
	m->tangent2 = b3Cross(m->tangent1, n);
}

void b3MeshContact::CollideSphere()
{
	b3SphereShape* sA = (b3SphereShape*)GetShapeA();
	b3Body* bodyA = sA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3MeshShape* sB = (b3MeshShape*)GetShapeB();
	b3Body* bodyB = sB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	b3World* world = bodyA->GetWorld();
	b3StackAllocator* allocator = &world->m_stackAllocator;

	b3SMCollider collider(allocator, m_triangles, m_triangleCount, xfA, sA, xfB, sB);
	m_manifoldCount = collider.Collide(m_stackManifolds);
}