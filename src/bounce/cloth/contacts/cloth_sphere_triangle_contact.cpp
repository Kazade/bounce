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

#include <bounce/cloth/contacts/cloth_sphere_triangle_contact.h>
#include <bounce/cloth/shapes/cloth_sphere_shape.h>
#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/common/geometry.h>

// Solve constrained Barycentric coordinates for point Q
static void b3Solve3(scalar out[3],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		out[0] = scalar(1);
		out[1] = scalar(0);
		out[2] = scalar(0);
		return;
	}

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		out[0] = scalar(0);
		out[1] = scalar(1);
		out[2] = scalar(0);
		return;
	}

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		out[0] = scalar(0);
		out[1] = scalar(0);
		out[2] = scalar(1);
		return;
	}

	// Test edge regions		
	scalar wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && wABC[3] * wABC[2] <= scalar(0))
	{
		scalar divisor = wAB[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		out[0] = s * wAB[0];
		out[1] = s * wAB[1];
		out[2] = scalar(0);
		return;
	}

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && wABC[3] * wABC[0] <= scalar(0))
	{
		scalar divisor = wBC[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		out[0] = scalar(0);
		out[1] = s * wBC[0];
		out[2] = s * wBC[1];
		return;
	}

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && wABC[3] * wABC[1] <= scalar(0))
	{
		scalar divisor = wCA[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		out[0] = s * wCA[1];
		out[1] = scalar(0);
		out[2] = s * wCA[0];
		return;
	}

	// R ABC/ACB
	scalar divisor = wABC[3];
	if (divisor == scalar(0))
	{
		scalar s = scalar(1) / scalar(3);
		out[0] = s;
		out[1] = s;
		out[2] = s;
		return;
	}

	B3_ASSERT(divisor > scalar(0));
	scalar s = scalar(1) / divisor;
	out[0] = s * wABC[0];
	out[1] = s * wABC[1];
	out[2] = s * wABC[2];
}

void b3ClothSphereAndTriangleContact::Update()
{
	m_active = false;
	
	b3ClothParticle* p1 = m_s1->m_p;
	b3ClothParticle* p2 = m_s2->m_p1;
	b3ClothParticle* p3 = m_s2->m_p2;
	b3ClothParticle* p4 = m_s2->m_p3;

	b3Vec3 A = p2->m_position;
	b3Vec3 B = p3->m_position;
	b3Vec3 C = p4->m_position;

	b3Vec3 N = b3Cross(B - A, C - A);
	scalar len = N.Normalize();

	// Is ABC degenerate?
	if (len == scalar(0))
	{
		return;
	}

	scalar r1 = m_s1->m_radius;
	scalar r2 = m_s2->m_radius;

	scalar totalRadius = r1 + r2;

	b3Vec3 P1 = p1->m_position;

	scalar distance = b3Dot(N, P1 - A);

	// Is P1 below the plane?
	if (distance < -totalRadius)
	{
		return;
	}

	// Is P1 above the plane?
	if (distance > totalRadius)
	{
		return;
	}

	// Closest point on ABC to P1
	scalar wABC[3];
	b3Solve3(wABC, A, B, C, P1);

	b3Vec3 P2 = wABC[0] * A + wABC[1] * B + wABC[2] * C;

	if (b3DistanceSquared(P1, P2) > totalRadius * totalRadius)
	{
		return;
	}

	// Activate the contact
	m_active = true;

	// Store Barycentric coordinates for P1
	m_w1 = wABC[0];
	m_w2 = wABC[1];
	m_w3 = wABC[2];

	b3Vec3 n = P2 - P1;
	if (n.Normalize() < B3_EPSILON)
	{
		n = N;
	}

	m_normal1 = n;
	m_tangent1 = b3Perp(m_normal1);
	m_tangent2 = b3Cross(m_tangent1, m_normal1);
}