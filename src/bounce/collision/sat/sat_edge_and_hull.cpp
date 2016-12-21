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

#include <bounce/collision/sat/sat_edge_and_hull.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>

float32 b3ProjectEdge(const b3Capsule* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xfA, const b3Capsule* hullA,
	const b3Transform& xfB, const b3Hull* hullB)
{
	// Perform computations in the local space of the first hull.
	b3Transform xf = b3MulT(xfA, xfB);

	// Here greater means less than since is a signed distance.
	u32 maxIndex = 0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	for (u32 i = 0; i < hullB->faceCount; ++i)
	{
		b3Plane plane = b3Mul(xf, hullB->GetPlane(i));
		float32 separation = b3ProjectEdge(hullA, plane);
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

// Qualify the two hull normals against the plane of the ring of the capsule.
bool b3IsMinkowskiFaceEdge(const b3Vec3& N, const b3Vec3& C, const b3Vec3& D)
{
	return b3Dot(N, C) * b3Dot(N, D) < 0.0f;
}

float32 b3ProjectEdge(const b3Vec3& P1, const b3Vec3& E1,
	const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C2)
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
	if (b3Dot(N, P2 - C2) > 0.0f)
	{
		N = -N;
	}

	// d = dot(N, P2) - offset = dot(N, P2) - dot(N, P1) = dot(N, P2 - P1) 
	return b3Dot(N, P2 - P1);
}

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xfA, const b3Capsule* hullA, const b3Transform& xfB, const b3Hull* hullB)
{
	// Query minimum edge separation.
	u32 maxIndex = 0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xfB, xfA);

	b3Vec3 P1 = b3Mul(xf, hullA->vertices[0]);
	b3Vec3 Q1 = b3Mul(xf, hullA->vertices[1]);
	b3Vec3 E1 = Q1 - P1;

	b3Vec3 C2 = hullB->centroid;

	for (u32 i = 0; i < hullB->edgeCount; i += 2)
	{
		const b3HalfEdge* edge2 = hullB->GetEdge(i);
		const b3HalfEdge* twin2 = hullB->GetEdge(i + 1);

		B3_ASSERT(edge2->twin == i + 1 && twin2->twin == i);

		b3Vec3 P2 = hullB->GetVertex(edge2->origin);
		b3Vec3 Q2 = hullB->GetVertex(twin2->origin);
		b3Vec3 E2 = Q2 - P2;

		b3Vec3 U2 = hullB->GetPlane(edge2->face).normal;
		b3Vec3 V2 = hullB->GetPlane(twin2->face).normal;

		if (b3IsMinkowskiFaceEdge(E1, U2, V2))
		{
			float32 separation = b3ProjectEdge(P1, E1, P2, E2, C2);
			if (separation > maxSeparation)
			{
				maxSeparation = separation;
				maxIndex = i;
			}
		}
	}

	b3EdgeQuery out;
	out.indexA = 0;
	out.indexB = maxIndex;
	out.separation = maxSeparation;
	return out;
}
