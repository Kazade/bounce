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

#include <bounce/cloth/contacts/cloth_capsule_capsule_contact.h>
#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/common/geometry.h>
#include <bounce/collision/shapes/capsule.h>

// Compute the closest point on a segment to a point. 
static b3Vec3 b3ClosestPointOnSegment(const b3Vec3& Q, const b3Capsule& hull)
{
	b3Vec3 A = hull.vertex1;
	b3Vec3 B = hull.vertex2;
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	if (v <= scalar(0))
	{
		return A;
	}

	if (u <= scalar(0))
	{
		return B;
	}

	scalar w = b3Dot(AB, AB);
	if (w <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		return A;
	}

	scalar den = scalar(1) / w;
	b3Vec3 P = den * (u * A + v * B);
	return P;
}

// Compute the closest points between two line segments.
static void b3ClosestPoints(b3Vec3& C1, b3Vec3& C2,
	const b3Capsule& hull1, const b3Capsule& hull2)
{
	b3Vec3 P1 = hull1.vertex1;
	b3Vec3 Q1 = hull1.vertex2;

	b3Vec3 P2 = hull2.vertex1;
	b3Vec3 Q2 = hull2.vertex2;

	b3Vec3 E1 = Q1 - P1;
	scalar L1 = b3Length(E1);

	b3Vec3 E2 = Q2 - P2;
	scalar L2 = b3Length(E2);

	if (L1 <= B3_LINEAR_SLOP && L2 <= B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = P2;
		return;
	}

	if (L1 <= B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = b3ClosestPointOnSegment(P1, hull2);
		return;
	}

	if (L2 <= B3_LINEAR_SLOP)
	{
		C1 = b3ClosestPointOnSegment(P2, hull1);
		C2 = P2;
		return;
	}

	B3_ASSERT(L1 > scalar(0));
	b3Vec3 N1 = E1 / L1;

	B3_ASSERT(L2 > scalar(0));
	b3Vec3 N2 = E2 / L2;

	scalar b = b3Dot(N1, N2);
	
	scalar den = scalar(1) - b * b;

	if (den != scalar(0))
	{
		// The segments are paralell.
		scalar inv_den = scalar(1) / den;

		b3Vec3 E3 = P1 - P2;

		scalar d = b3Dot(N1, E3);
		scalar e = b3Dot(N2, E3);

		scalar s = inv_den * (b * e - d);
		scalar t = inv_den * (e - b * d);

		C1 = P1 + s * N1;
		C2 = P2 + t * N2;
	}
	else
	{
		C1 = P1;
		C2 = P2;
	}

	C1 = b3ClosestPointOnSegment(C1, hull1);

	C2 = b3ClosestPointOnSegment(C1, hull2);

	C1 = b3ClosestPointOnSegment(C2, hull1);
}

void b3ClothCapsuleAndCapsuleContact::Update()
{
	m_active = false;

	b3ClothParticle* p1 = m_s1->m_p1;
	b3ClothParticle* p2 = m_s1->m_p2;
	b3ClothParticle* p3 = m_s2->m_p1;
	b3ClothParticle* p4 = m_s2->m_p2;

	b3Vec3 P1 = p1->m_position;
	b3Vec3 Q1 = p2->m_position;

	b3Vec3 P2 = p3->m_position;
	b3Vec3 Q2 = p4->m_position;

	b3Capsule hull1;
	hull1.vertex1 = P1;
	hull1.vertex2 = Q1;

	b3Capsule hull2;
	hull2.vertex1 = P2;
	hull2.vertex2 = Q2;

	b3Vec3 point1, point2;
	b3ClosestPoints(point1, point2, hull1, hull2);

	scalar distance = b3Distance(point1, point2);

	scalar r1 = m_s1->m_radius, r2 = m_s2->m_radius;
	
	scalar totalRadius = r1 + r2;
	
	if (distance > totalRadius)
	{
		return;
	}

	if (distance < scalar(10) * B3_EPSILON)
	{
		// The segments are intersecting.
		return;
	}

	scalar w12[3];
	b3BarycentricCoordinates(w12, P1, Q1, point1);
	if (w12[2] <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		m_w1 = scalar(1);
		m_w2 = scalar(0);
	}
	else
	{
		m_w1 = w12[0] / w12[2];
		m_w2 = w12[1] / w12[2];
	}

	scalar w23[3];
	b3BarycentricCoordinates(w23, P2, Q2, point2);
	if (w23[2] <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		m_w3 = scalar(1);
		m_w4 = scalar(0);
	}
	else
	{
		m_w3 = w23[0] / w23[2];
		m_w4 = w23[1] / w23[2];
	}

	// Activate the contact
	m_active = true;

	m_normal1 = (point2 - point1) / distance;
	m_tangent1 = b3Perp(m_normal1);
	m_tangent2 = b3Cross(m_tangent1, m_normal1);
}