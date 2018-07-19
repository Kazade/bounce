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
#include <bounce/collision/shapes/sphere.h>

void b3CollideSphereAndSphere(b3Manifold& manifold, 
	const b3Transform& xf1, const b3SphereShape* s1,
	const b3Transform& xf2, const b3SphereShape* s2)
{
	b3Vec3 c1 = xf1 * s1->m_center;
	float32 r1 = s1->m_radius;

	b3Vec3 c2 = xf2 * s2->m_center;
	float32 r2 = s2->m_radius;
	
	b3Vec3 d = c2 - c1;
	float32 dd = b3Dot(d, d);
	float32 totalRadius = r1 + r2;
	if (dd > totalRadius * totalRadius)
	{
		return;
	}

	float32 distance = b3Length(d);
	b3Vec3 normal(0.0f, 1.0f, 0.0f);
	if (distance > B3_EPSILON)
	{
		normal = d / distance;
	}
	
	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulT(xf1.rotation, normal);
	manifold.points[0].localPoint1 = s1->m_center;
	manifold.points[0].localPoint2 = s2->m_center;
	manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key.key1 = 0;
	manifold.points[0].key.key2 = 0;
}