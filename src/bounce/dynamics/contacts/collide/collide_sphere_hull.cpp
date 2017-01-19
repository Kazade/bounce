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
#include <bounce/dynamics/contacts/manifold.h>
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/hull.h>

void b3CollideSphereAndHull(b3Manifold& manifold,
	const b3Transform& xf1, const b3SphereShape* s1, 
	const b3Transform& xf2, const b3HullShape* s2) 
{
	b3ShapeGJKProxy proxy1(s1, 0);	
	b3ShapeGJKProxy proxy2(s2, 0);	
	b3GJKOutput distance = b3GJK(xf1, proxy1, xf2, proxy2);	
	float32 totalRadius = s1->m_radius + s2->m_radius;
	if (distance.distance > totalRadius)
	{
		return;
	}
	
	if (distance.distance > 0.0f)
	{
		b3Vec3 p1 = distance.pointA;
		b3Vec3 p2 = distance.pointB;
		b3Vec3 normal = (p2 - p1) / distance.distance;
		
		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = 0;
		manifold.points[0].localNormal = b3MulT(xf1.rotation, normal);
		manifold.points[0].localPoint = s1->m_center;
		manifold.points[0].localPoint2 = b3MulT(xf2, p2);
		
		manifold.center = 0.5f * (p1 + s1->m_radius * normal + p2 - s2->m_radius * normal);
		manifold.normal = normal;
		manifold.tangent1 = b3Perp(normal);
		manifold.tangent2 = b3Cross(manifold.tangent1, normal);

		return;
	}

	b3Sphere hull1;
	hull1.vertex = s1->m_center;
	hull1.radius = s1->m_radius;
	
	const b3Hull* hull2 = s2->m_hull;

	b3FaceQuery faceQuery = b3QueryFaceSeparation(xf1, &hull1, xf2, hull2);
	if (faceQuery.separation > totalRadius)
	{
		return;
	}

	b3Plane localPlane = hull2->GetPlane(faceQuery.index);
	b3Plane plane = b3Mul(xf2, localPlane);
	b3Vec3 cp1 = b3Mul(xf1, hull1.vertex);
	b3Vec3 cp2 = b3ClosestPointOnPlane(cp1, plane);
	
	// Ensure normal orientation to shape B
	b3Vec3 normal = -plane.normal;
	
	manifold.pointCount = 1;
	manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key = 1;
	manifold.points[0].localNormal = b3MulT(xf1.rotation, normal);
	manifold.points[0].localPoint = s1->m_center;
	manifold.points[0].localPoint2 = b3MulT(xf2, cp2);
	
	manifold.center = 0.5f * (cp1 + s1->m_radius * normal + cp2 - B3_HULL_RADIUS * normal);
	manifold.normal = normal;
	manifold.tangent1 = b3Perp(normal);
	manifold.tangent2 = b3Cross(manifold.tangent1, normal);
}