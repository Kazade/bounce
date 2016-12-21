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
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>

void b3CollideSphereAndCapsule(b3Manifold& manifold,
	const b3Transform& xfA, const b3SphereShape* sA,
	const b3Transform& xfB, const b3CapsuleShape* sB)
{
	b3Vec3 Q = b3Mul(xfA, sA->m_center);

	b3Vec3 A = b3Mul(xfB, sB->m_centers[0]);
	b3Vec3 B = b3Mul(xfB, sB->m_centers[1]);
	b3Vec3 AB = B - A;
	
	// Barycentric coordinates for Q
	float32 u = b3Dot(B - Q, AB);
	float32 v = b3Dot(Q - A, AB);
	
	float32 radius = sA->m_radius + sB->m_radius;

	if (v <= 0.0f)
	{
		// A
		b3Vec3 P = A;
		b3Vec3 d = P - Q;
		float32 dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(0.0f, 1.0f, 0.0f);
		float32 len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}
		
		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = 0;
		manifold.points[0].localNormal = b3MulT(xfA.rotation, n);
		manifold.points[0].localPoint = sA->m_center;
		manifold.points[0].localPoint2 = sB->m_centers[0];

		manifold.center = 0.5f * (P + sA->m_radius * n + Q - sB->m_radius * n);
		manifold.normal = n;
		manifold.tangent1 = b3Perp(n);
		manifold.tangent2 = b3Cross(manifold.tangent1, n);
		return;
	}

	if (u <= 0.0f)
	{
		// B
		b3Vec3 P = B;
		b3Vec3 d = P - Q;
		float32 dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(0.0f, 1.0f, 0.0f);
		float32 len = b3Length(d);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold.pointCount = 1;
		manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key = 0;
		manifold.points[0].localNormal = b3MulT(xfA.rotation, n);
		manifold.points[0].localPoint = sA->m_center;
		manifold.points[0].localPoint2 = sB->m_centers[1];

		manifold.center = 0.5f * (P + sA->m_radius * n + Q - sB->m_radius * n);
		manifold.normal = n;
		manifold.tangent1 = b3Perp(n);
		manifold.tangent2 = b3Cross(manifold.tangent1, n);
		return;
	}

	// AB
	//float32 s = u + v;
	float32 s = b3Dot(AB, AB);
	//B3_ASSERT(s > 0.0f);
	b3Vec3 P;
	if (s < B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		P = A;
	}
	else
	{
		P = (u * A + v * B) / s;
	}
	
	b3Vec3 d = P - Q;
	float32 dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}
	
	b3Vec3 QA = A - Q;
	b3Vec3 e = b3Cross(AB, QA);
	b3Vec3 n = b3Cross(AB, e);
	if (b3Dot(n, QA) < 0.0f)
	{
		n = -n;
	}
	n.Normalize();
	
	manifold.pointCount = 1;
	manifold.points[0].triangleKey = B3_NULL_TRIANGLE;
	manifold.points[0].key = 2;
	manifold.points[0].localNormal = b3MulT(xfA.rotation, n);
	manifold.points[0].localPoint = sA->m_center;
	manifold.points[0].localPoint2 = b3MulT(xfB, P);

	manifold.center = 0.5f * (P + sA->m_radius * n + Q - sB->m_radius * n);
	manifold.normal = n;
	manifold.tangent1 = b3Perp(n);
	manifold.tangent2 = b3Cross(manifold.tangent1, n);
}
