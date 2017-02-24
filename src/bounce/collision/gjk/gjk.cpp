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

#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/gjk/gjk_proxy.h>

// Implementation of the GJK (Gilbert-Johnson-Keerthi) algorithm 
// using Voronoi regions and Barycentric coordinates.

u32 b3_gjkCalls = 0, b3_gjkIters = 0, b3_gjkMaxIters = 0;

b3Vec3 b3Simplex::GetSearchDirection(const b3Vec3& Q) const
{
	switch (m_count)
	{
	case 1:
	{
		return Q - m_vertices[0].point;
	}
	case 2:
	{
		b3Vec3 A = m_vertices[0].point;
		b3Vec3 B = m_vertices[1].point;

		b3Vec3 AB = B - A;
		b3Vec3 AQ = Q - A;
		b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
		b3Vec3 PQ = b3Cross(AB_x_AQ, AB);
		return PQ;
	}
	case 3:
	{
		b3Vec3 A = m_vertices[0].point;
		b3Vec3 B = m_vertices[1].point;
		b3Vec3 C = m_vertices[2].point;

		b3Vec3 AB = B - A;
		b3Vec3 AC = C - A;
		b3Vec3 AQ = Q - A;
		b3Vec3 AB_x_AC = b3Cross(AB, AC);
		float32 sign = b3Dot(AB_x_AC, AQ);
		if (sign > 0.0f)
		{
			return AB_x_AC;
		}
		else
		{
			return -AB_x_AC;
		}
	}
	default:
	{
		B3_ASSERT(false);
		return b3Vec3(0.0f, 0.0f, 0.0f);
	}
	}
}

b3Vec3 b3Simplex::GetClosestPoint() const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		return b3Vec3(0.0f, 0.0f, 0.0f);
	case 1:
		return m_vertices[0].point;
	case 2:
		return m_vertices[0].weight * m_vertices[0].point + m_vertices[1].weight * m_vertices[1].point;
	case 3:
		return m_vertices[0].weight * m_vertices[0].point + m_vertices[1].weight * m_vertices[1].point + m_vertices[2].weight * m_vertices[2].point;
	case 4:
		return b3Vec3(0.0f, 0.0f, 0.0f);
	default:
		B3_ASSERT(false);
		return b3Vec3(0.0f, 0.0f, 0.0f);
	}
}

void b3Simplex::GetClosestPoints(b3Vec3* pA, b3Vec3* pB) const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		break;
	case 1:
		*pA = m_vertices[0].pointA;
		*pB = m_vertices[0].pointB;
		break;

	case 2:
		*pA = m_vertices[0].weight * m_vertices[0].pointA + m_vertices[1].weight * m_vertices[1].pointA;
		*pB = m_vertices[0].weight * m_vertices[0].pointB + m_vertices[1].weight * m_vertices[1].pointB;
		break;

	case 3:
		*pA = m_vertices[0].weight * m_vertices[0].pointA + m_vertices[1].weight * m_vertices[1].pointA + m_vertices[2].weight * m_vertices[2].pointA;
		*pB = m_vertices[0].weight * m_vertices[0].pointB + m_vertices[1].weight * m_vertices[1].pointB + m_vertices[2].weight * m_vertices[2].pointB;
		break;
	case 4:
		*pA = m_vertices[0].weight * m_vertices[0].pointA + m_vertices[1].weight * m_vertices[1].pointA + m_vertices[2].weight * m_vertices[2].pointA + m_vertices[3].weight * m_vertices[3].pointA;
		*pB = *pA;
		break;
	default:
		B3_ASSERT(false);
		break;
	}
}

// Closest point on edge AB to Q.
void b3Simplex::Solve2(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];

	// Test vertex regions
	float32 wAB[3];
	b3Barycentric(wAB, A.point, B.point, Q);

	// R A
	if (wAB[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R B
	if (wAB[0] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R AB
	float32 divisor = wAB[2];
	float32 s = 1.0f / divisor;
	m_count = 2;
	m_vertices[0] = A;
	m_vertices[0].weight = s * wAB[0];
	m_vertices[1] = B;
	m_vertices[1].weight = s * wAB[1];
}

// Closest point on face ABC to Q.
// Voronoi regions: A, B, C, AB, AC, BC, ABC, ACB.
void b3Simplex::Solve3(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];
	b3SimplexVertex C = m_vertices[2];

	// Test vertex regions
	float32 wAB[3], wBC[3], wCA[3];
	b3Barycentric(wAB, A.point, B.point, Q);
	b3Barycentric(wBC, B.point, C.point, Q);
	b3Barycentric(wCA, C.point, A.point, Q);
	
	// Test edge regions		
	float32 wABC[4];
	b3Barycentric(wABC, A.point, B.point, C.point, Q);

	// Test vertex regions
	// R A
	if (wAB[1] <= 0.0f && wCA[0] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R B
	if (wAB[0] <= 0.0f && wBC[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R C
	if (wBC[0] <= 0.0f && wCA[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = C;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// Test edge regions		
	// This is used to help testing if the face degenerates 
	// into an edge.
	float32 area = wABC[3];

	// R AB
	if (wAB[0] > 0.0f && wAB[1] > 0.0f && area * wABC[2] <= 0.0f)
	{
		float32 divisor = wAB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wAB[1];
		return;
	}

	// R BC
	if (wBC[0] > 0.0f && wBC[1] > 0.0f && area * wABC[0] <= 0.0f)
	{
		float32 divisor = wBC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBC[1];
		return;
	}

	// R CA
	if (wCA[0] > 0.0f && wCA[1] > 0.0f && area * wABC[1] <= 0.0f)
	{
		float32 divisor = wCA[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = C;
		m_vertices[0].weight = s * wCA[0];
		m_vertices[1] = A;
		m_vertices[1].weight = s * wCA[1];
		return;
	}

	// R ABC/ACB
	float32 divisor = wABC[3];
	if (divisor <= 0.0f)
	{
		// Give up.
		return;
	}

	B3_ASSERT(divisor > 0.0f);
	float32 s = 1.0f / divisor;
	m_count = 3;
	m_vertices[0] = A;
	m_vertices[0].weight = s * wABC[0];
	m_vertices[1] = B;
	m_vertices[1].weight = s * wABC[1];
	m_vertices[2] = C;
	m_vertices[2].weight = s * wABC[2];
}

// Closest point on tetrahedron A, B, C, D to Q.
void b3Simplex::Solve4(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];
	b3SimplexVertex C = m_vertices[2];
	b3SimplexVertex D = m_vertices[3];

	// Test vertex regions
	float32 wAB[3], wAC[3], wAD[3], wBC[3], wCD[3], wDB[3];
	b3Barycentric(wAB, A.point, B.point, Q);
	b3Barycentric(wBC, B.point, C.point, Q);
	b3Barycentric(wAC, A.point, C.point, Q);
	b3Barycentric(wAD, A.point, D.point, Q);
	b3Barycentric(wCD, C.point, D.point, Q);
	b3Barycentric(wDB, D.point, B.point, Q);

	// Test edge regions
	float32 wACB[4], wABD[4], wADC[4], wBCD[4];
	b3Barycentric(wACB, A.point, C.point, B.point, Q);
	b3Barycentric(wABD, A.point, B.point, D.point, Q);
	b3Barycentric(wADC, A.point, D.point, C.point, Q);
	b3Barycentric(wBCD, B.point, C.point, D.point, Q);
	
	// Test face regions
	float32 wABCD[5];
	b3Barycentric(wABCD, A.point, B.point, C.point, D.point, Q);

	// Test vertex regions
	
	// R A
	if (wAB[1] <= 0.0f && wAC[1] <= 0.0f && wAD[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R B
	if (wAB[0] <= 0.0f && wDB[0] <= 0.0f && wBC[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R C
	if (wAC[0] <= 0.0f && wBC[0] <= 0.0f && wCD[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = C;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// R D
	if (wAD[0] <= 0.0f && wCD[0] <= 0.0f && wDB[1] <= 0.0f)
	{
		m_count = 1;
		m_vertices[0] = D;
		m_vertices[0].weight = 1.0f;
		return;
	}

	// Test edge regions
	
	// R AB
	if (wABD[2] <= 0.0f && wACB[1] <= 0.0f && wAB[0] > 0.0f && wAB[1] > 0.0f)
	{
		float32 divisor = wAB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wAB[1];
		return;
	}

	// R AC
	if (wACB[2] <= 0.0f && wADC[1] <= 0.0f && wAC[0] > 0.0f && wAC[1] > 0.0f)
	{
		float32 divisor = wAC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wAC[1];
		return;
	}

	// R AD
	if (wADC[2] <= 0.0f && wABD[1] <= 0.0f && wAD[0] > 0.0f && wAD[1] > 0.0f)
	{
		float32 divisor = wAD[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAD[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wAD[1];
		return;
	}

	// R BC
	if (wACB[0] <= 0.0f && wBCD[2] <= 0.0f && wBC[0] > 0.0f && wBC[1] > 0.0f)
	{
		float32 divisor = wBC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBC[1];
		return;
	}

	// R CD
	if (wADC[0] <= 0.0f && wBCD[0] <= 0.0f && wCD[0] > 0.0f && wCD[1] > 0.0f)
	{
		float32 divisor = wCD[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = C;
		m_vertices[0].weight = s * wCD[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wCD[1];
		return;
	}

	// R DB
	if (wABD[0] <= 0.0f && wBCD[1] <= 0.0f &&  wDB[0] > 0.0f && wDB[1] > 0.0f)
	{
		float32 divisor = wDB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 2;
		m_vertices[0] = D;
		m_vertices[0].weight = s * wDB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wDB[1];
		return;
	}

	// Test face regions
	
	// R ACB 
	if (wABCD[3] <= 0.0f && wACB[0] > 0.0f && wACB[1] > 0.0f && wACB[2] > 0.0f)
	{
		float32 divisor = wACB[0] + wACB[1] + wACB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wACB[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wACB[1];
		m_vertices[2] = B;
		m_vertices[2].weight = s * wACB[2];
		return;
	}

	// R ABD
	if (wABCD[2] <= 0.0f && wABD[0] > 0.0f && wABD[1] > 0.0f && wABD[2] > 0.0f)
	{
		float32 divisor = wABD[0] + wABD[1] + wABD[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wABD[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wABD[1];
		m_vertices[2] = D;
		m_vertices[2].weight = s * wABD[2];
		return;
	}

	// R ADC
	if (wABCD[1] <= 0.0f && wADC[0] > 0.0f && wADC[1] > 0.0f && wADC[2] > 0.0f)
	{
		float32 divisor = wADC[0] + wADC[1] + wADC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wADC[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wADC[1];
		m_vertices[2] = C;
		m_vertices[2].weight = s * wADC[2];
		return;
	}

	// R BCD
	if (wABCD[0] <= 0.0f && wBCD[0] > 0.0f && wBCD[1] > 0.0f && wBCD[2] > 0.0f)
	{
		float32 divisor = wBCD[0] + wBCD[1] + wBCD[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		m_count = 3;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBCD[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBCD[1];
		m_vertices[2] = D;
		m_vertices[2].weight = s * wBCD[2];
		return;
	}

	// R ABCD
	float32 divisor = wABCD[0] + wABCD[1] + wABCD[2] + wABCD[3];
	if (divisor <= 0.0f)
	{
		// Give up.
		return;
	}

	B3_ASSERT(divisor > 0.0f);
	float32 s = 1.0f / divisor;
	m_count = 4;
	m_vertices[0].weight = s * wABCD[0];
	m_vertices[1].weight = s * wABCD[1];
	m_vertices[2].weight = s * wABCD[2];
	m_vertices[3].weight = s * wABCD[3];
}

b3GJKOutput b3GJK(const b3Transform& xfA, const b3GJKProxy& proxyA, const b3Transform& xfB, const b3GJKProxy& proxyB)
{
	++b3_gjkCalls;

	b3Simplex simplex;
	
	// Initialize the simplex.
	{
		b3SimplexVertex* v = simplex.m_vertices + 0;
		b3Vec3 wALocal = proxyA.GetVertex(0);
		b3Vec3 wBLocal = proxyB.GetVertex(0);
		v->pointA = b3Mul(xfA, wALocal);
		v->pointB = b3Mul(xfB, wBLocal);
		v->point = v->pointB - v->pointA;
		v->weight = 1.0f;
		v->indexA = 0;
		v->indexB = 0;
		simplex.m_count = 1;
	}

	// Get simplex vertices as an array.
	b3SimplexVertex* vertices = simplex.m_vertices;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	u32 saveA[4], saveB[4];
	u32 saveCount = 0;

	// Last iteration squared distance for checking if we're getting close
	// to the origin and prevent cycling.
	float32 distSq1 = B3_MAX_FLOAT;

	const b3Vec3 kOrigin(0.0f, 0.0f, 0.0f);

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 20;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (u32 i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		// Determine the closest point on the simplex and
		// remove unused vertices.
		switch (simplex.m_count)
		{
		case 1:
			break;
		case 2:
			simplex.Solve2(kOrigin);
			break;
		case 3:
			simplex.Solve3(kOrigin);
			break;
		case 4:
			simplex.Solve4(kOrigin);
			break;
		default:
			B3_ASSERT(false);
			break;
		}

		// If we have 4 points, then the origin is in the corresponding tethrahedron.
		if (simplex.m_count == 4)
		{
			break;
		}

		// Compute the closest point.
		b3Vec3 p = simplex.GetClosestPoint();
		float32 distSq2 = b3Dot(p, p);
		// Ensure we're getting close to the origin.
		if (distSq2 >= distSq1)
		{
			//break;
		}
		distSq1 = distSq2;

		// Get search direction.
		b3Vec3 d = simplex.GetSearchDirection(kOrigin);

		// Ensure the search direction is non-zero.
		if (b3Dot(d, d) < B3_EPSILON * B3_EPSILON)
		{
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b3SimplexVertex* vertex = vertices + simplex.m_count;
		vertex->indexA = proxyA.GetSupportIndex(b3MulT(xfA.rotation, -d));
		vertex->pointA = b3Mul(xfA, proxyA.GetVertex(vertex->indexA));
		vertex->indexB = proxyB.GetSupportIndex(b3MulT(xfB.rotation, d));
		vertex->pointB = b3Mul(xfB, proxyB.GetVertex(vertex->indexB));
		vertex->point = vertex->pointB - vertex->pointA;

		// Iteration count is equated to the number of support point calls.
		++iter;
		++b3_gjkIters;

		// Check for duplicate support points. 
		// This is the main termination criteria.
		bool duplicate = false;
		for (u32 i = 0; i < saveCount; ++i)
		{
			if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is needed.
		++simplex.m_count;
	}

	b3_gjkMaxIters = b3Max(b3_gjkMaxIters, iter);

	// Prepare output.
	b3GJKOutput output;
	simplex.GetClosestPoints(&output.pointA, &output.pointB);
	output.distance = b3Distance(output.pointA, output.pointB);
	output.iterations = iter;

	// Output result.
	return output;
}
