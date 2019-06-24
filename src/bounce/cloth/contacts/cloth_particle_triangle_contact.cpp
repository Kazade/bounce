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

#include <bounce/cloth/contacts/cloth_particle_triangle_contact.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/common/geometry.h>

// Solve constrained Barycentric coordinates for point Q
static void b3Solve3(float32 out[3],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	// Test vertex regions
	float32 wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= 0.0f && wCA[0] <= 0.0f)
	{
		out[0] = 1.0f;
		out[1] = 0.0f;
		out[2] = 0.0f;
		return;
	}

	// R B
	if (wAB[0] <= 0.0f && wBC[1] <= 0.0f)
	{
		out[0] = 0.0f;
		out[1] = 1.0f;
		out[2] = 0.0f;
		return;
	}

	// R C
	if (wBC[0] <= 0.0f && wCA[1] <= 0.0f)
	{
		out[0] = 0.0f;
		out[1] = 0.0f;
		out[2] = 1.0f;
		return;
	}

	// Test edge regions		
	float32 wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > 0.0f && wAB[1] > 0.0f && wABC[3] * wABC[2] <= 0.0f)
	{
		float32 divisor = wAB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = s * wAB[0];
		out[1] = s * wAB[1];
		out[2] = 0.0f;
		return;
	}

	// R BC
	if (wBC[0] > 0.0f && wBC[1] > 0.0f && wABC[3] * wABC[0] <= 0.0f)
	{
		float32 divisor = wBC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = 0.0f;
		out[1] = s * wBC[0];
		out[2] = s * wBC[1];
		return;
	}

	// R CA
	if (wCA[0] > 0.0f && wCA[1] > 0.0f && wABC[3] * wABC[1] <= 0.0f)
	{
		float32 divisor = wCA[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = s * wCA[1];
		out[1] = 0.0f;
		out[2] = s * wCA[0];
		return;
	}

	// R ABC/ACB
	float32 divisor = wABC[3];
	if (divisor == 0.0f)
	{
		float32 s = 1.0f / 3.0f;
		out[0] = s;
		out[1] = s;
		out[2] = s;
		return;
	}

	B3_ASSERT(divisor > 0.0f);
	float32 s = 1.0f / divisor;
	out[0] = s * wABC[0];
	out[1] = s * wABC[1];
	out[2] = s * wABC[2];
}

void b3ParticleTriangleContact::Update()
{
	b3Vec3 A = m_p2->m_position;
	b3Vec3 B = m_p3->m_position;
	b3Vec3 C = m_p4->m_position;

	b3Vec3 N = b3Cross(B - A, C - A);
	float32 len = N.Normalize();

	// Is ABC degenerate?
	if (len == 0.0f)
	{
		m_active = false;
		return;
	}

	float32 r1 = m_p1->m_radius;
	float32 r2 = m_t2->m_radius;

	float32 totalRadius = r1 + r2;

	b3Vec3 P1 = m_p1->m_position;

	float32 distance = b3Dot(N, P1 - A);

	// Is P1 below the plane?
	if (distance < -totalRadius)
	{
		m_active = false;
		return;
	}

	// Is P1 above the plane?
	if (distance > totalRadius)
	{
		m_active = false;
		return;
	}

	// Closest point on ABC to P1
	float32 wABC[3];
	b3Solve3(wABC, A, B, C, P1);

	b3Vec3 P2 = wABC[0] * A + wABC[1] * B + wABC[2] * C;

	if (b3DistanceSquared(P1, P2) > totalRadius * totalRadius)
	{
		m_active = false;
		return;
	}

	// Activate the contact
	m_active = true;

	// Store Barycentric coordinates for P1
	m_w2 = wABC[0];
	m_w3 = wABC[1];
	m_w4 = wABC[2];
}