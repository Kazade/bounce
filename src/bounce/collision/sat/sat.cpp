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

#include <bounce/collision/sat/sat.h>
#include <bounce/collision/shapes/hull.h>

// Implementation of the SAT (Separating Axis Test) for 
// convex hulls. Thanks to Dirk Gregorius for his presentation 
// at GDC 2013.

float32 b3Project(const b3Hull* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

// Query minimum separation distance and axis of the first hull planes.
b3FaceQuery b3QueryFaceSeparation(const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xfB, xfA);

	// Here greater means less than since is a signed distance.
	u32 maxIndex = 0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	for (u32 i = 0; i < hullA->faceCount; ++i)
	{
		b3Plane plane = xf * hullA->GetPlane(i);
		float32 separation = b3Project(hullB, plane);
		if (separation > maxSeparation)
		{
			maxIndex = i;
			maxSeparation = separation;
		}
	}

	b3FaceQuery out;
	out.index = maxIndex;
	out.separation = maxSeparation;
	return out;
}

bool b3IsMinkowskiFace(const b3Vec3& A, const b3Vec3& B, const b3Vec3& B_x_A, const b3Vec3& C, const b3Vec3& D, const b3Vec3& D_x_C)
{
	float32 ADC = b3Dot(A, D_x_C);
	float32 BDC = b3Dot(B, D_x_C);

	float32 CBA = b3Dot(C, B_x_A);
	float32 DBA = b3Dot(D, B_x_A);

	return CBA * DBA < 0.0f && // Test arc CD against AB plane.
		ADC * BDC < 0.0f && // Test arc AB against DC plane.
		CBA * BDC > 0.0f; // Test if arcs AB and CD are on the same hemisphere.
}

float32 b3Project(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C1)
{
	// Skip over almost parallel edges.
	b3Vec3 E1_x_E2 = b3Cross(E1, E2);
	float32 L = b3Length(E1_x_E2);

	const float32 kTol = 0.005f;

	if (L < kTol * b3Sqrt(b3LengthSquared(E1) * b3LengthSquared(E2)))
	{
		return -B3_MAX_FLOAT;
	}

	// Ensure consistent normal orientation to hull B.
	b3Vec3 N = (1.0f / L) * E1_x_E2;
	if (b3Dot(N, P1 - C1) < 0.0f)
	{
		N = -N;
	}

	// d = dot(N, P2) - offset = dot(N, P2) - dot(N, P1) = dot(N, P2 - P1) 
	return b3Dot(N, P2 - P1);
}

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	// Query minimum separation distance and axis of the first hull planes.
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xfB, xfA);
	b3Vec3 C1 = xf * hullA->centroid;

	u32 maxIndex1 = 0;
	u32 maxIndex2 = 0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	// Loop through the first hull's unique edges.
	for (u32 i = 0; i < hullA->edgeCount; i += 2)
	{
		const b3HalfEdge* edge1 = hullA->GetEdge(i);
		const b3HalfEdge* twin1 = hullA->GetEdge(i + 1);

		B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

		b3Vec3 P1 = xf * hullA->GetVertex(edge1->origin);
		b3Vec3 Q1 = xf * hullA->GetVertex(twin1->origin);
		b3Vec3 E1 = Q1 - P1;

		// The Gauss Map of edge 1.
		b3Vec3 U1 = xf.rotation * hullA->GetPlane(edge1->face).normal;
		b3Vec3 V1 = xf.rotation * hullA->GetPlane(twin1->face).normal;

		// Loop through the second hull's unique edges.
		for (u32 j = 0; j < hullB->edgeCount; j += 2)
		{
			const b3HalfEdge* edge2 = hullB->GetEdge(j);
			const b3HalfEdge* twin2 = hullB->GetEdge(j + 1);

			B3_ASSERT(edge2->twin == j + 1 && twin2->twin == j);

			b3Vec3 P2 = hullB->GetVertex(edge2->origin);
			b3Vec3 Q2 = hullB->GetVertex(twin2->origin);
			b3Vec3 E2 = Q2 - P2;

			// The Gauss Map of edge 2.
			b3Vec3 U2 = hullB->GetPlane(edge2->face).normal;
			b3Vec3 V2 = hullB->GetPlane(twin2->face).normal;

			// Negate the Gauss Map 2 for account for the MD.
			if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2))
			{
				float32 separation = b3Project(P1, E1, P2, E2, C1);
				if (separation > maxSeparation)
				{
					maxSeparation = separation;
					maxIndex1 = i;
					maxIndex2 = j;
				}
			}
		}
	}

	b3EdgeQuery out;
	out.indexA = maxIndex1;
	out.indexB = maxIndex2;
	out.separation = maxSeparation;
	return out;
}
