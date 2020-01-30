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
#include <bounce/dynamics/shapes/triangle_shape.h>
#include <bounce/dynamics/shapes/sphere_shape.h>

void b3CollideTriangleAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* s1,
	const b3Transform& xf2, const b3SphereShape* s2)
{
	// Put the sphere center in the frame of the triangle.
	b3Vec3 Q = b3MulT(xf1, b3Mul(xf2, s2->m_center));

	// ABC
	b3Vec3 A = s1->m_vertex1, B = s1->m_vertex2, C = s1->m_vertex3;

	scalar radius = s1->m_radius + s2->m_radius;

	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(0, 1, 0);
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

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(0, 1, 0);
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

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		b3Vec3 P = C;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		b3Vec3 n(0, 1, 0);
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

	// Test edge regions		
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wABC[3] * wABC[2] <= scalar(0))
	{
		B3_ASSERT(wAB[2] > scalar(0));
		scalar den = scalar(1) / wAB[2];

		b3Vec3 P = den * (wAB[0] * A + wAB[1] * B);
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there a face connected to AB?
		if (s1->m_hasE1Vertex)
		{
			b3Vec3 A1 = s1->m_e1Vertex;
			b3Vec3 B1 = B;
			b3Vec3 C1 = A;

			scalar wABC1[4];
			b3BarycentricCoordinates(wABC1, A1, B1, C1, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC1[0] > scalar(0) && wABC1[1] > scalar(0) && wABC1[2] > scalar(0))
			{
				return;
			}
		}

		b3Vec3 n(0, 1, 0);
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

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && wABC[3] * wABC[0] <= scalar(0))
	{
		B3_ASSERT(wBC[2] > scalar(0));
		scalar den = scalar(1) / wBC[2];

		b3Vec3 P = den * (wBC[0] * B + wBC[1] * C);
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there a face connected to AC?
		if (s1->m_hasE2Vertex)
		{
			b3Vec3 A1 = s1->m_e2Vertex;
			b3Vec3 B1 = C;
			b3Vec3 C1 = B;

			scalar wABC1[4];
			b3BarycentricCoordinates(wABC1, A1, B1, C1, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC1[0] > scalar(0) && wABC1[1] > scalar(0) && wABC1[2] > scalar(0))
			{
				return;
			}
		}

		b3Vec3 n(0, 1, 0);
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

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && wABC[3] * wABC[1] <= scalar(0))
	{
		B3_ASSERT(wCA[2] > scalar(0));
		scalar den = scalar(1) / wCA[2];

		b3Vec3 P = den * (wCA[0] * C + wCA[1] * A);
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there a face connected to CA?
		if (s1->m_hasE3Vertex)
		{
			b3Vec3 A1 = s1->m_e3Vertex;
			b3Vec3 B1 = A;
			b3Vec3 C1 = C;

			scalar wABC1[4];
			b3BarycentricCoordinates(wABC1, A1, B1, C1, Q);

			// Is the sphere in the Region ABC of the adjacent face?
			if (wABC1[0] > scalar(0) && wABC1[1] > scalar(0) && wABC1[2] > scalar(0))
			{
				return;
			}
		}

		b3Vec3 n(0, 1, 0);
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

	// R ABC/ACB
	if (wABC[3] == scalar(0))
	{
		return;
	}

	B3_ASSERT(wABC[3] > scalar(0));
	scalar den = scalar(1) / wABC[3];
	
	b3Vec3 P = den * (wABC[0] * A + wABC[1] * B + wABC[2] * C);
	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}

	b3Vec3 n(0, 1, 0);
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
}