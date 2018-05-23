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
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/collision/collision.h>

void b3ShapeGJKProxy::Set(const b3Shape* shape, u32 index)
{
	switch (shape->GetType())
	{
	case e_sphereShape:
	{
		const b3SphereShape* sphere = (b3SphereShape*)shape;
		vertexCount = 1;
		vertices = &sphere->m_center;
		radius = sphere->m_radius;
		break;
	}
	case e_capsuleShape:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		vertexCount = 2;
		vertices = capsule->m_centers;
		radius = capsule->m_radius;
		break;
	}
	case e_hullShape:
	{
		const b3HullShape* hull = (b3HullShape*)shape;
		vertexCount = hull->m_hull->vertexCount;
		vertices = hull->m_hull->vertices;
		radius = hull->m_radius;
		break;
	}
	case e_meshShape:
	{
		const b3MeshShape* mesh = (b3MeshShape*)shape;

		B3_ASSERT(index >= 0);
		B3_ASSERT(index < mesh->m_mesh->triangleCount);

		const b3Triangle& triangle = mesh->m_mesh->GetTriangle(index);

		vertexBuffer[0] = mesh->m_mesh->vertices[triangle.v1];
		vertexBuffer[1] = mesh->m_mesh->vertices[triangle.v2];
		vertexBuffer[2] = mesh->m_mesh->vertices[triangle.v3];

		vertexCount = 3;
		vertices = vertexBuffer;
		radius = mesh->m_radius;
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

bool b3TestOverlap(const b3Transform& xfA, u32 indexA, const b3Shape* shapeA,
	const b3Transform& xfB, u32 indexB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	b3ShapeGJKProxy proxyA(shapeA, indexA);
	b3ShapeGJKProxy proxyB(shapeB, indexB);
	
	b3GJKOutput distance = b3GJK(xfA, proxyA, xfB, proxyB, true, &cache->simplexCache);

	const float32 kTol = 10.0f * B3_EPSILON;
	return distance.distance <= kTol;
}

void b3CollideSphereAndSphereShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3SphereShape* hullA = (b3SphereShape*)shapeA;
	b3SphereShape* hullB = (b3SphereShape*)shapeB;
	b3CollideSphereAndSphere(manifold, xfA, hullA, xfB, hullB);
}

void b3CollideSphereAndHullShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3SphereShape* hullA = (b3SphereShape*)shapeA;
	b3HullShape* hullB = (b3HullShape*)shapeB;
	b3CollideSphereAndHull(manifold, xfA, hullA, xfB, hullB);
}

void b3CollideSphereAndCapsuleShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3SphereShape* hullA = (b3SphereShape*)shapeA;
	b3CapsuleShape* hullB = (b3CapsuleShape*)shapeB;
	b3CollideSphereAndCapsule(manifold, xfA, hullA, xfB, hullB);
}

void b3CollideCapsuleAndCapsuleShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3CapsuleShape* hullA = (b3CapsuleShape*)shapeA;
	b3CapsuleShape* hullB = (b3CapsuleShape*)shapeB;
	b3CollideCapsuleAndCapsule(manifold, xfA, hullA, xfB, hullB);
}

void b3CollideCapsuleAndHullShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
    B3_NOT_USED(cache);
	b3CapsuleShape* hullA = (b3CapsuleShape*)shapeA;
	b3HullShape* hullB = (b3HullShape*)shapeB;
	b3CollideCapsuleAndHull(manifold, xfA, hullA, xfB, hullB);
}

void b3CollideHullAndHullShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	b3HullShape* hullA = (b3HullShape*)shapeA;
	b3HullShape* hullB = (b3HullShape*)shapeB;
	b3CollideHullAndHull(manifold, xfA, hullA, xfB, hullB, cache);
}

void b3CollideShapeAndShape(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB, 
	b3ConvexCache* cache)
{
	typedef void(*b3CollideFunction)(b3Manifold&, 
		const b3Transform&, const b3Shape*,
		const b3Transform&, const b3Shape*,
		b3ConvexCache*);

	static const b3CollideFunction s_CollideMatrix[e_maxShapes][e_maxShapes] =
	{
		{ &b3CollideSphereAndSphereShapes,	&b3CollideSphereAndCapsuleShapes,	&b3CollideSphereAndHullShapes  },
		{ NULL,							&b3CollideCapsuleAndCapsuleShapes,	&b3CollideCapsuleAndHullShapes },
		{ NULL,							NULL,							&b3CollideHullAndHullShapes	   },
	};

	b3ShapeType typeA = shapeA->GetType();
	b3ShapeType typeB = shapeB->GetType();

	B3_ASSERT(typeA <= typeB);
	
	b3CollideFunction CollideFunc = s_CollideMatrix[typeA][typeB];
	
	B3_ASSERT(CollideFunc);
	CollideFunc(manifold, xfA, shapeA, xfB, shapeB, cache);
}