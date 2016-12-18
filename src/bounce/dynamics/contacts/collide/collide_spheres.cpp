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

#include <bounce\dynamics\contacts\collide\collide.h>
#include <bounce\dynamics\contacts\manifold.h>
#include <bounce\dynamics\shapes\sphere_shape.h>
#include <bounce\collision\shapes\sphere.h>

void b3CollideSphereAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3SphereShape* s1,
	const b3Transform& xf2, const b3SphereShape* s2)
{
	b3Vec3 c1 = b3Mul(xf1, s1->m_center);
	b3Vec3 c2 = b3Mul(xf2, s2->m_center);
	b3Vec3 d = c2 - c1;
	float32 dd = b3Dot(d, d);
	float32 totalRadius = s1->m_radius + s2->m_radius;
	if (dd > totalRadius * totalRadius)
	{
		return;
	}

	float32 distance = b3Length(d);
	
	b3Vec3 normal;
	if (distance > B3_EPSILON)
	{
		normal = d / distance;
	}
	else
	{
		normal.Set(0.0f, 1.0f, 0.0f);
	}
	
	b3Vec3 point = 0.5f * (c1 + s1->m_radius * normal + c2 - s2->m_radius * normal);
	
	manifold.pointCount = 1;
	manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key = 0;
	manifold.points[0].localNormal = b3MulT(xf1.rotation, normal);
	manifold.points[0].localPoint = s1->m_center;
	manifold.points[0].localPoint2 = s2->m_center;

	manifold.center = point;
	manifold.normal = normal;
	manifold.tangent1 = b3Perp(normal);
	manifold.tangent2 = b3Cross(manifold.tangent1, normal);
}