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
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/triangle_hull.h>

// Half-edge to edge map
struct b3TriangleHullEdges
{
	bool IsEdgeCoplanar(u32 index) const;
	
	const b3TriangleHull* m_triangleHull;
	bool m_hasWing[3];
	b3Vec3 m_edgeWings[3];
	u32 m_halfEdgeEdges[6];
};

bool b3TriangleHullEdges::IsEdgeCoplanar(u32 halfEdgeIndex) const
{
	u32 edgeIndex = m_halfEdgeEdges[halfEdgeIndex];

	if (m_hasWing[edgeIndex] == false)
	{
		return false;
	}
	
	u32 ev1 = edgeIndex;
	u32 ev2 = edgeIndex + 1 < 3 ? edgeIndex + 1 : 0;

	// Adjacent triangle
	b3Vec3 A1 = m_edgeWings[edgeIndex];
	b3Vec3 B1 = m_triangleHull->triangleVertices[ev2];
	b3Vec3 C1 = m_triangleHull->triangleVertices[ev1];
	
	b3Vec3 center1 = (A1 + B1 + C1) / scalar(3);

	b3Plane frontPlane = m_triangleHull->trianglePlanes[0];

	scalar d = b3Distance(center1, frontPlane);

	const scalar kCoplanarTol = 0.005f;

	if (d > -kCoplanarTol && d < kCoplanarTol)
	{
		return true;
	}

	return false;
}

void b3CollideTriangleAndHull(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* s1,
	const b3Transform& xf2, const b3HullShape* s2,
	b3ConvexCache* cache)
{
	const b3TriangleHull triangleHull1(s1->m_vertex1, s1->m_vertex2, s1->m_vertex3);
	const b3Hull* hull2 = s2->m_hull;

	b3HullShape hullShape1;
	hullShape1.SetShape((b3Shape*)s1);
	hullShape1.m_hull = &triangleHull1;
	hullShape1.m_radius = s1->m_radius;

	b3CollideHullAndHull(manifold, xf1, &hullShape1, xf2, s2, cache);

	b3TriangleHullEdges triangleHullEdges;
	triangleHullEdges.m_triangleHull = &triangleHull1;
	
	triangleHullEdges.m_hasWing[0] = s1->m_hasE1Vertex;
	triangleHullEdges.m_hasWing[1] = s1->m_hasE2Vertex;
	triangleHullEdges.m_hasWing[2] = s1->m_hasE3Vertex;

	triangleHullEdges.m_edgeWings[0] = s1->m_e1Vertex;
	triangleHullEdges.m_edgeWings[1] = s1->m_e2Vertex;
	triangleHullEdges.m_edgeWings[2] = s1->m_e3Vertex;

	triangleHullEdges.m_halfEdgeEdges[0] = 0;
	triangleHullEdges.m_halfEdgeEdges[2] = 1;
	triangleHullEdges.m_halfEdgeEdges[4] = 2;

	triangleHullEdges.m_halfEdgeEdges[1] = 0;
	triangleHullEdges.m_halfEdgeEdges[3] = 1;
	triangleHullEdges.m_halfEdgeEdges[5] = 2;

	b3Vec3 C1 = xf1 * triangleHull1.centroid;
	b3Plane plane1 = xf1 * triangleHull1.planes[0];
	
	b3Vec3 C2 = xf2 * hull2->centroid;

	for (u32 i = 0; i < manifold.pointCount; ++i)
	{
		b3ManifoldPoint* mp = manifold.points + i;

		b3FeaturePair pair = mp->featurePair;

		u32 e1;
		if (mp->edgeContact)
		{
			e1 = pair.inEdge1;
		}
		else
		{
			if (pair.inEdge1 == B3_NULL_EDGE)
			{
				e1 = pair.outEdge1;
			}
			else
			{
				e1 = pair.inEdge1;
			}

			if (e1 == B3_NULL_EDGE)
			{
				continue;
			}
		}

		b3Vec3 localN1 = mp->localNormal1;
		b3Vec3 localC1 = mp->localPoint1;
		b3Vec3 localC2 = mp->localPoint2;

		b3Vec3 n1 = b3Mul(xf1.rotation, localN1);
		b3Vec3 c1 = xf1 * localC1;
		b3Vec3 c2 = xf2 * localC2;
		scalar s = b3Dot(c2 - c1, n1);

		if (triangleHullEdges.IsEdgeCoplanar(e1))
		{
			b3Vec3 n = plane1.normal;
			
			if (b3Dot(n, C2 - C1) < scalar(0))
			{
				n = -n;
			}

			// c1 is constant
			// c2 = c1 + s * n1
			// c1 = c2 - s * n1
			b3Vec3 nc2 = c1 + s * n;

			mp->localNormal1 = b3MulC(xf1.rotation, n);
			mp->localPoint2 = b3MulT(xf2, nc2);
		}
	}
}