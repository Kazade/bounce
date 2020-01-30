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
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/triangle_shape.h>
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
		vertices = &capsule->m_vertex1;
		radius = capsule->m_radius;
		break;
	}
	case e_triangleShape:
	{
		const b3TriangleShape* triangle = (b3TriangleShape*)shape;
		vertexCount = 3;
		vertices = &triangle->m_vertex1;
		radius = triangle->m_radius;
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

		const b3MeshTriangle* triangle = mesh->m_mesh->GetTriangle(index);

		vertexBuffer[0] = b3MulCW(mesh->m_scale, mesh->m_mesh->vertices[triangle->v1]);
		vertexBuffer[1] = b3MulCW(mesh->m_scale, mesh->m_mesh->vertices[triangle->v2]);
		vertexBuffer[2] = b3MulCW(mesh->m_scale, mesh->m_mesh->vertices[triangle->v3]);

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

	const scalar kTol = scalar(10) * B3_EPSILON;
	return distance.distance <= kTol;
}

void b3CollideSphereAndSphereShapes(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3SphereShape* sphereA = (b3SphereShape*)shapeA;
	b3SphereShape* sphereB = (b3SphereShape*)shapeB;
	b3CollideSphereAndSphere(manifold, xfA, sphereA, xfB, sphereB);
}

void b3CollideCapsuleAndSphereShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3CapsuleShape* capsuleA = (b3CapsuleShape*)shapeA;
	b3SphereShape* sphereB = (b3SphereShape*)shapeB;
	b3CollideCapsuleAndSphere(manifold, xfA, capsuleA, xfB, sphereB);
}

void b3CollideCapsuleAndCapsuleShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3CapsuleShape* capsuleA = (b3CapsuleShape*)shapeA;
	b3CapsuleShape* capsuleB = (b3CapsuleShape*)shapeB;
	b3CollideCapsuleAndCapsule(manifold, xfA, capsuleA, xfB, capsuleB);
}

void b3CollideTriangleAndSphereShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3TriangleShape* triangleA = (b3TriangleShape*)shapeA;
	b3SphereShape* sphereB = (b3SphereShape*)shapeB;
	b3CollideTriangleAndSphere(manifold, xfA, triangleA, xfB, sphereB);
}

void b3CollideTriangleAndCapsuleShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3TriangleShape* triangleA = (b3TriangleShape*)shapeA;
	b3CapsuleShape* capsuleB = (b3CapsuleShape*)shapeB;
	b3CollideTriangleAndCapsule(manifold, xfA, triangleA, xfB, capsuleB);
}

void b3CollideTriangleAndHullShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	b3TriangleShape* triangleA = (b3TriangleShape*)shapeA;
	b3HullShape* hullB = (b3HullShape*)shapeB;
	b3CollideTriangleAndHull(manifold, xfA, triangleA, xfB, hullB, cache);
}

void b3CollideHullAndSphereShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3HullShape* hullA = (b3HullShape*)shapeA;
	b3SphereShape* sphereB = (b3SphereShape*)shapeB;
	b3CollideHullAndSphere(manifold, xfA, hullA, xfB, sphereB);
}

void b3CollideHullAndCapsuleShapes(b3Manifold& manifold,
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	B3_NOT_USED(cache);
	b3HullShape* hullA = (b3HullShape*)shapeA;
	b3CapsuleShape* capsuleB = (b3CapsuleShape*)shapeB;
	b3CollideHullAndCapsule(manifold, xfA, hullA, xfB, capsuleB);
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

typedef void(*b3CollideFcn)(b3Manifold&,
	const b3Transform&, const b3Shape*,
	const b3Transform&, const b3Shape*,
	b3ConvexCache*);

static b3CollideFcn s_functions[e_maxShapes][e_maxShapes];

static void b3SetCollideFunction(b3ShapeType typeA, b3ShapeType typeB, b3CollideFcn fcn)
{
	B3_ASSERT(0 <= typeA && typeA < e_maxShapes);
	B3_ASSERT(0 <= typeB && typeB < e_maxShapes);

	s_functions[typeA][typeB] = fcn;
}

static void b3InitializeCollideFunctions()
{
	b3SetCollideFunction(e_sphereShape, e_sphereShape, &b3CollideSphereAndSphereShapes);

	b3SetCollideFunction(e_capsuleShape, e_sphereShape, &b3CollideCapsuleAndSphereShapes);
	b3SetCollideFunction(e_capsuleShape, e_capsuleShape, &b3CollideCapsuleAndCapsuleShapes);

	b3SetCollideFunction(e_triangleShape, e_sphereShape, &b3CollideTriangleAndSphereShapes);
	b3SetCollideFunction(e_triangleShape, e_capsuleShape, &b3CollideTriangleAndCapsuleShapes);
	b3SetCollideFunction(e_triangleShape, e_hullShape, &b3CollideTriangleAndHullShapes);

	b3SetCollideFunction(e_hullShape, e_capsuleShape, &b3CollideHullAndCapsuleShapes);
	b3SetCollideFunction(e_hullShape, e_sphereShape, &b3CollideHullAndSphereShapes);
	b3SetCollideFunction(e_hullShape, e_hullShape, &b3CollideHullAndHullShapes);
}

void b3CollideShapeAndShape(b3Manifold& manifold, 
	const b3Transform& xfA, const b3Shape* shapeA,
	const b3Transform& xfB, const b3Shape* shapeB, 
	b3ConvexCache* cache)
{
	static bool b3Collide_initilized = false;
	if (b3Collide_initilized == false)
	{
		b3InitializeCollideFunctions();
		b3Collide_initilized = true;
	}

	b3ShapeType typeA = shapeA->GetType();
	b3ShapeType typeB = shapeB->GetType();

	b3CollideFcn fcn = s_functions[typeA][typeB];
	
	B3_ASSERT(fcn != nullptr);
	fcn(manifold, xfA, shapeA, xfB, shapeB, cache);
}