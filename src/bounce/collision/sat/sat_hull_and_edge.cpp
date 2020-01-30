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

#include <bounce/collision/sat/sat_hull_and_edge.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/capsule.h>

scalar b3ProjectEdge(const b3Capsule* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Capsule* hull2)
{
	// Perform computations in the local space of the first hull.
	b3Transform xf = b3MulT(xf1, xf2);

	b3Capsule localHull2;
	localHull2.vertex1 = xf * hull2->vertex1;
	localHull2.vertex2 = xf * hull2->vertex2;

	// Here greater means less than since is a signed distance.
	u32 maxIndex = 0;
	scalar maxSeparation = -B3_MAX_SCALAR;

	for (u32 i = 0; i < hull1->faceCount; ++i)
	{
		b3Plane plane = hull1->GetPlane(i);
		scalar separation = b3ProjectEdge(&localHull2, plane);
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
	return b3Dot(N, C) * b3Dot(N, D) < scalar(0);
}

scalar b3ProjectEdge(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& C1,
	const b3Vec3& P2, const b3Vec3& E2)
{
	scalar L1 = b3Length(E1);
	if (L1 < B3_LINEAR_SLOP)
	{
		return -B3_MAX_SCALAR;
	}

	scalar L2 = b3Length(E2);
	if (L2 < B3_LINEAR_SLOP)
	{
		return -B3_MAX_SCALAR;
	}

	// Skip over almost parallel edges.
	const scalar kTol = 0.005f;

	b3Vec3 E1_x_E2 = b3Cross(E1, E2);
	scalar L = b3Length(E1_x_E2);
	if (L < kTol * L1 * L2)
	{
		return -B3_MAX_SCALAR;
	}

	// Ensure consistent normal orientation to hull B.
	b3Vec3 N = (scalar(1) / L) * E1_x_E2;
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	// d = dot(N, P2) - offset = dot(N, P2) - dot(N, P1) = dot(N, P2 - P1) 
	return b3Dot(N, P2 - P1);
}

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xf1, const b3Hull* hull1, const b3Transform& xf2, const b3Capsule* hull2)
{
	// Query minimum edge separation.
	u32 maxIndex = 0;
	scalar maxSeparation = -B3_MAX_SCALAR;

	// Perform computations in the local space of the first hull.
	b3Transform xf = b3MulT(xf1, xf2);
	b3Vec3 C1 = hull1->centroid;

	b3Vec3 P2 = xf * hull2->vertex1;
	b3Vec3 Q2 = xf * hull2->vertex2;
	b3Vec3 E2 = Q2 - P2;

	for (u32 i = 0; i < hull1->edgeCount; i += 2)
	{
		const b3HalfEdge* edge1 = hull1->GetEdge(i);
		const b3HalfEdge* twin1 = hull1->GetEdge(i + 1);

		B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

		b3Vec3 P1 = hull1->GetVertex(edge1->origin);
		b3Vec3 Q1 = hull1->GetVertex(twin1->origin);
		b3Vec3 E1 = Q1 - P1;

		b3Vec3 U1 = hull1->GetPlane(edge1->face).normal;
		b3Vec3 V1 = hull1->GetPlane(twin1->face).normal;

		if (b3IsMinkowskiFaceEdge(E2, U1, V1))
		{
			scalar separation = b3ProjectEdge(P1, E1, C1, P2, E2);
			if (separation > maxSeparation)
			{
				maxSeparation = separation;
				maxIndex = i;
			}
		}
	}

	b3EdgeQuery out;
	out.index1 = maxIndex;
	out.index2 = B3_MAX_U32;
	out.separation = maxSeparation;
	return out;
}