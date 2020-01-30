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
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/sphere_shape.h>

void b3CollideCapsuleAndSphere(b3Manifold& manifold, 
	const b3Transform& xf1, const b3CapsuleShape* s1,
	const b3Transform& xf2, const b3SphereShape* s2)
{
	// The sphere center in the frame of the capsule.
	b3Vec3 Q = b3MulT(xf1, b3Mul(xf2, s2->m_center));

	b3Vec3 A = s1->m_vertex1, B = s1->m_vertex2;
	
	b3Vec3 AB = B - A;
	
	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);
	
	scalar radius = s1->m_radius + s2->m_radius;

	if (v <= scalar(0))
	{
		// A
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}
		
		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = n;
		manifold.points[0].localPoint1 = P;
		manifold.points[0].localPoint2 = s2->m_center;
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}

	if (u <= scalar(0))
	{
		// B
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = n;
		manifold.points[0].localPoint1 = P;
		manifold.points[0].localPoint2 = s2->m_center;
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}

	// AB
	scalar s = b3Dot(AB, AB);
	B3_ASSERT(s > scalar(0));
	b3Vec3 P = (u * A + v * B) / s;

	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}
	
	b3Vec3 AQ = Q - A;
	b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
	b3Vec3 n = b3Cross(AB_x_AQ, AB);
	if (b3Dot(n, AQ) < scalar(0))
	{
		n = -n;
	}
	n.Normalize();
	
	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = n;
	manifold.points[0].localPoint1 = P;
	manifold.points[0].localPoint2 = s2->m_center;
	manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key.key1 = 0;
	manifold.points[0].key.key2 = 0;
}