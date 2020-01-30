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
#include <bounce/dynamics/contacts/manifold.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/sphere.h>

void b3CollideHullAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* s1,
	const b3Transform& xf2, const b3SphereShape* s2)
{
	scalar radius = s1->m_radius + s2->m_radius;
	const b3Hull* hull1 = s1->m_hull;

	// Sphere center in the frame of the hull.
	b3Vec3 cLocal = b3MulT(xf1, b3Mul(xf2, s2->m_center));

	// Find the minimum separation face.	
	u32 faceIndex = 0;
	scalar separation = -B3_MAX_SCALAR;

	for (u32 i = 0; i < hull1->faceCount; ++i)
	{
		b3Plane plane = hull1->GetPlane(i);
		scalar s = b3Distance(cLocal, plane);

		if (s > radius)
		{
			// Early out.
			return;
		}

		if (s > separation)
		{
			faceIndex = i;
			separation = s;
		}
	}

	if (separation < scalar(0))
	{
		// The center is inside the hull.
		b3Plane localPlane1 = hull1->planes[faceIndex];

		b3Vec3 c1 = b3ClosestPointOnPlane(cLocal, localPlane1);

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = localPlane1.normal;
		manifold.points[0].localPoint1 = c1;
		manifold.points[0].localPoint2 = s2->m_center;
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;
		return;
	}

	// Reference face polygon.
	b3StackArray<b3Vec3, 64> referencePolygon;

	const b3Face* face = hull1->GetFace(faceIndex);
	const b3HalfEdge* begin = hull1->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		b3Vec3 vertex = hull1->GetVertex(edge->origin);
		referencePolygon.PushBack(vertex);
		edge = hull1->GetEdge(edge->next);
	} while (edge != begin);

	b3GJKProxy proxy1;
	proxy1.vertexCount = referencePolygon.Count();
	proxy1.vertices = referencePolygon.Begin();

	b3GJKProxy proxy2;
	proxy2.vertexBuffer[0] = s2->m_center;
	proxy2.vertexCount = 1;
	proxy2.vertices = proxy2.vertexBuffer;

	b3GJKOutput query = b3GJK(xf1, proxy1, xf2, proxy2, false);

	if (query.distance > radius)
	{
		return;
	}

	if (query.distance > scalar(0))
	{
		b3Vec3 c1 = query.point1;
		b3Vec3 c2 = query.point2;
		scalar d = query.distance;

		b3Vec3 normal = (c2 - c1) / d;

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulC(xf1.rotation, normal);
		manifold.points[0].localPoint1 = b3MulT(xf1, c1);
		manifold.points[0].localPoint2 = s2->m_center;
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;
	}
}