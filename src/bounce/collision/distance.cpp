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

#include <bounce/collision/distance.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat22.h>

b3Vec3 b3ClosestPointOnPlane(const b3Vec3& P, const b3Plane& plane)
{
	return b3Project(P, plane);
}

b3Vec3 b3ClosestPointOnSegment(const b3Vec3& P,
	const b3Vec3& A, const b3Vec3& B)
{
	float32 wAB[3];
	b3Barycentric(wAB, A, B, P);

	if (wAB[1] <= 0.0f)
	{
		return A;
	}

	if (wAB[0] <= 0.0f)
	{
		return B;
	}

	float32 s = 1.0f / wAB[2];
	float32 wA = s * wAB[0];
	float32 wB = s * wAB[1];
	return wA * A + wB * B;
}

b3Vec3 b3ClosestPointOnTriangle(const b3Vec3& P,
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
{
	// Test vertex regions
	float32 wAB[3], wBC[3], wCA[3];
	b3Barycentric(wAB, A, B, P);
	b3Barycentric(wBC, B, C, P);
	b3Barycentric(wCA, C, A, P);

	// R A
	if (wAB[1] <= 0.0f && wCA[0] <= 0.0f)
	{
		return A;
	}

	// R B
	if (wAB[0] <= 0.0f && wBC[2] <= 0.0f)
	{
		return B;
	}

	// R C
	if (wBC[0] <= 0.0f && wCA[1] <= 0.0f)
	{
		return C;
	}

	// Test edge regions
	float32 wABC[4];
	b3Barycentric(wABC, A, B, C, P);

	// R AB
	if (wAB[0] > 0.0f && wAB[1] > 0.0f && wABC[2] <= 0.0f)
	{
		float32 s = 1.0f / wAB[2];
		float32 wA = s * wAB[0];
		float32 wB = s * wAB[1];
		return wA * A + wB * B;
	}

	// R BC
	if (wBC[0] > 0.0f && wBC[1] > 0.0f && wABC[0] <= 0.0f)
	{
		float32 s = 1.0f / wBC[2];
		float32 wB = s * wBC[0];
		float32 wC = s * wBC[1];
		return wB * B + wC * C;
	}

	// R CA
	if (wCA[0] > 0.0f && wCA[1] > 0.0f && wABC[1] <= 0.0f)
	{
		float32 s = 1.0f / wCA[2];
		float32 wC = s * wCA[0];
		float32 wA = s * wCA[1];
		return wC * C + wA * A;
	}

	if (wABC[3] <= 0.0f)
	{
		// ABC degenerates into an edge or vertex.
		// Give up.
		return A;
	}

	// R ABC/ACB
	float32 divisor = wABC[3];
	float32 s = 1.0f / divisor;
	float32 wA = s * wABC[0];
	float32 wB = s * wABC[1];
	float32 wC = s * wABC[2];
	return wA * A + wB * B + wC * C;
}

void b3ClosestPointsOnLines(b3Vec3* C1, b3Vec3* C2,
	const b3Vec3& P1, const b3Vec3& E1,
	const b3Vec3& P2, const b3Vec3& E2)
{
	// E1 = Q1 - P1
	// E2 = Q2 - P2
	// S1 = P1 + x1 * E1
	// S2 = P2 + x2 * E2
	// [dot(E1, E1) -dot(E1, E2)][x1] = [-dot(E1, P1 - P2)] 
	// [dot(E2, E1) -dot(E2, E2)][x2] = [-dot(E2, P1 - P2)]
	// Ax = b
	b3Vec3 E3 = P1 - P2;

	b3Mat22 A;
	A.x.x = b3Dot(E1, E1);
	A.x.y = b3Dot(E2, E1);
	A.y.x = -b3Dot(E1, E2);
	A.y.y = -b3Dot(E2, E2);

	b3Vec2 b;
	b.x = -b3Dot(E1, E3);
	b.y = -b3Dot(E2, E3);

	// If the lines are parallel then choose P1 and P2 as 
	// the closest points.
	b3Vec2 x = A.Solve(b);

	*C1 = P1 + x.x * E1;
	*C2 = P2 + x.y * E2;
}

void b3ClosestPointsOnNormalizedLines(b3Vec3* C1, b3Vec3* C2,
	const b3Vec3& P1, const b3Vec3& N1,
	const b3Vec3& P2, const b3Vec3& N2)
{
	const float32 kTol = 0.001f;

	float32 b = b3Dot(N1, N2);
	float32 den = 1.0f - b * b;
	if (den < kTol * kTol)
	{
		*C1 = P1;
		*C2 = P2;
		return;
	}

	float32 inv_den = 1.0f / den;
	
	b3Vec3 E3 = P1 - P2;

	float32 d = b3Dot(N1, E3);
	float32 e = b3Dot(N2, E3);

	float32 s = inv_den * (b * e - d);
	float32 t = inv_den * (e - b * d);

	*C1 = P1 + s * N1;
	*C2 = P2 + t * N2;
}

void b3ClosestPointsOnSegments(b3Vec3* C1, b3Vec3* C2,
	const b3Vec3& P1, const b3Vec3& Q1,
	const b3Vec3& P2, const b3Vec3& Q2)
{
	b3Vec3 E1 = Q1 - P1;
	float32 L1 = b3Length(E1);

	b3Vec3 E2 = Q2 - P2;
	float32 L2 = b3Length(E2);

	if (L1 < B3_LINEAR_SLOP && L2 < B3_LINEAR_SLOP)
	{
		*C1 = P1;
		*C2 = P2;
		return;
	}

	if (L1 < B3_LINEAR_SLOP)
	{
		*C1 = P1;
		*C2 = L2 < B3_LINEAR_SLOP ? P2 : b3ClosestPointOnSegment(P1, P2, Q2);
		return;
	}

	if (L2 < B3_LINEAR_SLOP)
	{
		*C1 = L1 < B3_LINEAR_SLOP ? P1 : b3ClosestPointOnSegment(P2, P1, Q1);
		*C2 = P2;
		return;
	}

	// Here and in 3D we need to start "GJK" with the closest points between the two edges 
	// since the cross product between their direction is a possible separating axis.
	b3Vec3 N1 = (1.0f / L1) * E1;
	b3Vec3 N2 = (1.0f / L2) * E2;

	// sin = norm( cross(n1, n2) )
	// or
	// sin^2 = 1 - cos^2

	// Use small parallelism tolerance because the coincidence tolerance above is also smal.
	const float32 kTol = 0.005f;

	float32 b = b3Dot(N1, N2);
	float32 den = 1.0f - b * b;
	if (den < kTol * kTol)
	{
		*C1 = P1;
		*C2 = P2;
	}
	else
	{
		// b = dot(n1, n2)
		// d = dot(n1, e3)
		// e = dot(n2, e3)

		// s - b * t = -d
		// b * s - t = -e

		// s = ( b * e - d ) / den 
		// t = ( e - b * d ) / den 
		b3Vec3 E3 = P1 - P2;

		float32 d = b3Dot(N1, E3);
		float32 e = b3Dot(N2, E3);
		float32 inv_den = 1.0f / den;

		float32 s = inv_den * (b * e - d);
		float32 t = inv_den * (e - b * d);

		*C1 = P1 + s * N1;
		*C2 = P2 + t * N2;
	}

	*C1 = b3ClosestPointOnSegment(*C1, P1, Q1);
	*C2 = b3ClosestPointOnSegment(*C1, P2, Q2);
	*C1 = b3ClosestPointOnSegment(*C2, P1, Q1);
}
