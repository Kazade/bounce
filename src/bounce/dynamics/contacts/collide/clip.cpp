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

#include <bounce/dynamics/contacts/collide/clip.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>

void b3BuildEdge(b3ClipVertex vOut[2],
	const b3Capsule* hull)
{
	vOut[0].position = hull->vertices[0];
	vOut[0].pair = b3MakePair(0, B3_NULL_EDGE, 0, B3_NULL_EDGE);
	
	vOut[1].position = hull->vertices[1];
	vOut[1].pair = b3MakePair(1, B3_NULL_EDGE, 0, B3_NULL_EDGE);
}

void b3BuildPolygon(b3ClipPolygon& pOut,
	const b3Transform& xf, u32 index, const b3Hull* hull)
{
	B3_ASSERT(pOut.Count() == 0);

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);
		u32 edgeId = twin->twin;
		u32 previousEdgeId = twin->next;

		b3ClipVertex clipVertex;
		clipVertex.position = b3Mul(xf, hull->GetVertex(edge->origin));
		clipVertex.pair = b3MakePair(previousEdgeId, B3_NULL_EDGE, edgeId, B3_NULL_EDGE);

		pOut.PushBack(clipVertex);

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	B3_ASSERT(pOut.Count() > 2);
}

// Sutherland-Hodgman clipping.
u32 b3ClipEdgeToPlane(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3ClipPlane& plane)
{
	u32 numOut = 0;

	float32 distance1 = b3Distance(vIn[0].position, plane.plane);
	float32 distance2 = b3Distance(vIn[1].position, plane.plane);

	// If the points are behind the plane keep them
	if (distance1 <= 0.0f)
	{
		vOut[numOut++] = vIn[0];
	}

	if (distance2 <= 0.0f)
	{
		vOut[numOut++] = vIn[1];
	}

	// If the points are on opposite sides keep intersection
	if (distance1 <= 0.0f && distance2 > 0.0f)
	{
		float32 fraction = distance1 / (distance1 - distance2);
		vOut[numOut].position = vIn[0].position + fraction * (vIn[1].position - vIn[0].position);
		vOut[numOut].pair = b3MakePair(vIn[0].pair.inEdge1, B3_NULL_EDGE, B3_NULL_EDGE, plane.id);
		++numOut;
	}
	else if (distance1 > 0.0f && distance2 <= 0.0f)
	{
		float32 fraction = distance1 / (distance1 - distance2);
		vOut[numOut].position = vIn[0].position + fraction * (vIn[1].position - vIn[0].position);
		vOut[numOut].pair = b3MakePair(vIn[1].pair.inEdge1, plane.id, B3_NULL_EDGE, B3_NULL_EDGE);
		++numOut;
	}

	return numOut;
}

// Sutherland-Hodgman clipping.
void b3ClipPolygonToPlane(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3ClipPlane& plane)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);

	b3ClipVertex v1 = pIn.Back();
	float32 distance1 = b3Distance(v1.position, plane.plane);

	for (u32 i = 0; i < pIn.Count(); ++i)
	{
		b3ClipVertex v2 = pIn[i];
		float32 distance2 = b3Distance(v2.position, plane.plane);

		if (distance1 <= 0.0f && distance2 <= 0.0f)
		{
			// Both vertices are behind or lying on the plane.
			// Keep v2
			pOut.PushBack(v2);
		}
		else if (distance1 <= 0.0f && distance2 > 0.0f)
		{
			// v1 is behind and v2 in front
			// Keep intersection
			float32 fraction = distance1 / (distance1 - distance2);

			b3ClipVertex vertex;
			vertex.position = v1.position + fraction * (v2.position - v1.position);
			vertex.pair = b3MakePair(v1.pair.inEdge1, B3_NULL_EDGE, B3_NULL_EDGE, plane.id);

			pOut.PushBack(vertex);
		}
		else if (distance1 > 0.0f && distance2 <= 0.0f)
		{
			// v2 is behind and v1 in front
			// Keep intersection and v2
			float32 fraction = distance1 / (distance1 - distance2);

			b3ClipVertex vertex;
			vertex.position = v1.position + fraction * (v2.position - v1.position);
			vertex.pair = b3MakePair(v1.pair.inEdge1, plane.id, B3_NULL_EDGE, B3_NULL_EDGE);

			pOut.PushBack(vertex);
			pOut.PushBack(v2);
		}

		// Make v2 as the starting vertex of the next edge
		v1 = v2;
		distance1 = distance2;
	}
}

// Clip a segment to edge side planes.
u32 b3ClipEdgeToFace(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Capsule* hull)
{
	// Start from somewhere.
	vOut[0] = vIn[0];
	vOut[1] = vIn[1];
	u32 numOut = 0;

	b3Vec3 P1 = hull->vertices[0];
	b3Vec3 Q1 = hull->vertices[1];
	b3Vec3 E1 = Q1 - P1;

	B3_ASSERT(b3Dot(E1, E1) > B3_EPSILON * B3_EPSILON);
	b3ClipPlane clipPlane1;
	clipPlane1.plane.normal = b3Normalize(E1);
	clipPlane1.plane.offset = b3Dot(clipPlane1.plane.normal, Q1);
	clipPlane1.id = 0;

	b3ClipVertex clipEdge1[2];
	numOut = b3ClipEdgeToPlane(clipEdge1, vOut, clipPlane1);

	vOut[0] = clipEdge1[0];
	vOut[1] = clipEdge1[1];

	if (numOut < 2)
	{
		return numOut;
	}

	b3ClipPlane clipPlane2;
	clipPlane2.plane.normal = -clipPlane1.plane.normal;
	clipPlane2.plane.offset = b3Dot(clipPlane2.plane.normal, P1);
	clipPlane2.id = 1;

	b3ClipVertex clipEdge2[2];
	numOut = b3ClipEdgeToPlane(clipEdge2, vOut, clipPlane2);

	vOut[0] = clipEdge2[0];
	vOut[1] = clipEdge2[1];

	if (numOut < 2)
	{
		return numOut;
	}

	return numOut;
}

// Clip a segment to face side planes.
u32 b3ClipEdgeToFace(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Transform& xf, float32 r, u32 index, const b3Hull* hull)
{
	// Start from somewhere.
	vOut[0] = vIn[0];
	vOut[1] = vIn[1];
	u32 numOut = 0;

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);
		u32 edgeId = u32(twin->twin);

		b3Plane plane = hull->GetEdgeSidePlane(edgeId);
		plane.offset += r;

		b3ClipPlane clipPlane;
		clipPlane.id = edgeId;
		clipPlane.plane = b3Mul(xf, plane);

		b3ClipVertex clipEdge[2];
		numOut = b3ClipEdgeToPlane(clipEdge, vOut, clipPlane);

		vOut[0] = clipEdge[0];
		vOut[1] = clipEdge[1];

		if (numOut == 0)
		{
			return numOut;
		}

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	// Now vOut contains the clipped points.
	return numOut;
}

// Clip a polygon to face side planes.
void b3ClipPolygonToFace(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3Transform& xf, float32 r, u32 index, const b3Hull* hull)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);

	// Start from somewhere.
	pOut = pIn;

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);
		u32 edgeId = u32(twin->twin);

		b3Plane plane = hull->GetEdgeSidePlane(edgeId);
		plane.offset += r;

		b3ClipPlane clipPlane;
		clipPlane.id = edgeId;
		clipPlane.plane = b3Mul(xf, plane);

		b3StackArray<b3ClipVertex, 32> clipPolygon;
		b3ClipPolygonToPlane(clipPolygon, pOut, clipPlane);
		pOut = clipPolygon;

		if (pOut.IsEmpty())
		{
			return;
		}

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	// Now pOut contains the clipped points.
}