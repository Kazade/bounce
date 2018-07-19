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
	
	b3GJKOutput gjk = b3GJK(xf1, proxy1, xf2, proxy2);	
		
	float32 r1 = s1->m_radius;
	float32 r2 = s2->m_radius;

	float32 totalRadius = r1 + r2;
	
	if (gjk.distance > totalRadius)
	{
		return;
	}
	
	if (gjk.distance > 0.0f)
	{
		b3Vec3 c1 = gjk.point1;
		b3Vec3 c2 = gjk.point2;
		b3Vec3 normal = (c2 - c1) / gjk.distance;
		
		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulT(xf1.rotation, normal);
		manifold.points[0].localPoint1 = s1->m_center;
		manifold.points[0].localPoint2 = b3MulT(xf2, c2);
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}

	const b3Sphere hull1(s1->m_center, 0.0f);
	const b3Hull* hull2 = s2->m_hull;

	b3FaceQuery faceQuery = b3QueryFaceSeparation(xf1, &hull1, xf2, hull2);
	if (faceQuery.separation > totalRadius)
	{
		return;
	}

	b3Plane localPlane2 = hull2->planes[faceQuery.index];
	b3Plane plane2 = xf2 * localPlane2;
	
	b3Vec3 c1 = xf1 * hull1.vertex;
	b3Vec3 c2 = b3ClosestPointOnPlane(c1, plane2);
	
	// Ensure normal orientation to shape 2
	b3Vec3 n1 = -plane2.normal;

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulT(xf1.rotation, n1);
	manifold.points[0].localPoint1 = s1->m_center;
	manifold.points[0].localPoint2 = b3MulT(xf2, c2);
	manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key.key1 = 0;
	manifold.points[0].key.key2 = 0;
}