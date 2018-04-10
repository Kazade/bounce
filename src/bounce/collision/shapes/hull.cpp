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

#include <bounce/collision/shapes/hull.h>

void b3Hull::Validate() const 
{
	for (u32 i = 0; i < faceCount; ++i) 
	{
		Validate(faces + i);
	}

	for (u32 i = 0; i < edgeCount; i += 2)
	{
		const b3HalfEdge* edge = edges + i;
		const b3HalfEdge* twin = edges + i + 1;

		b3Vec3 A = vertices[edge->origin];
		b3Vec3 B = vertices[twin->origin];

		// Ensure each edge has non-zero length.
		B3_ASSERT(b3DistanceSquared(A, B) > B3_LINEAR_SLOP * B3_LINEAR_SLOP);
	}
}

void b3Hull::Validate(const b3Face* face) const 
{
	bool ok = false;
	const b3HalfEdge* begin = GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		if (GetFace(edge->face) == face)
		{
			ok = true;
			break;
		}
		edge = edges + edge->next;
	} while (edge != begin);

	B3_ASSERT(ok);

	Validate(edges + face->edge);
}

void b3Hull::Validate(const b3HalfEdge* e) const 
{
	u8 edgeIndex = (u8)(e - edges);
	
	const b3HalfEdge* twin = edges + e->twin;

	// Each edge must be followed by its twin.
	B3_ASSERT(b3Abs(twin - e) == 1);
	B3_ASSERT(twin->twin == edgeIndex);

	u32 count = 0;
	const b3HalfEdge* begin = e;
	do 
	{
		const b3HalfEdge* next = edges + e->next;
		e = edges + next->twin;
		B3_ASSERT(count < edgeCount);
		++count;
	} while (e != begin);
}

b3Vec3 b3Hull::GetCentroid() const
{
	Validate();

	b3Vec3 c(0.0f, 0.0f, 0.0f);
	float32 volume = 0.0f;

	// Pick reference point not too away from the origin 
	// to minimize floating point rounding errors.
	b3Vec3 p1(0.0f, 0.0f, 0.0f);
	// Put it inside the hull.
	for (u32 i = 0; i < vertexCount; ++i)
	{
		p1 += vertices[i];
	}
	p1 /= float32(vertexCount);

	const float32 inv4 = 0.25f;
	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv60 = 1.0f / 60.0f;
	const float32 inv120 = 1.0f / 120.0f;

	b3Vec3 diag(0.0f, 0.0f, 0.0f);
	b3Vec3 offDiag(0.0f, 0.0f, 0.0f);

	// Triangulate convex polygons
	for (u32 i = 0; i < faceCount; ++i)
	{
		const b3Face* face = GetFace(i);
		const b3HalfEdge* begin = GetEdge(face->edge);

		const b3HalfEdge* edge = GetEdge(begin->next);
		do
		{
			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			const b3HalfEdge* next = GetEdge(edge->next);
			u32 i3 = next->origin;

			b3Vec3 p2 = vertices[i1];
			b3Vec3 p3 = vertices[i2];
			b3Vec3 p4 = vertices[i3];

			b3Vec3 e1 = p2 - p1;
			b3Vec3 e2 = p3 - p1;
			b3Vec3 e3 = p4 - p1;

			float32 D = b3Det(e1, e2, e3);

			float32 tetraVolume = inv6 * D;
			volume += tetraVolume;

			// Volume weighted centroid
			c += tetraVolume * inv4 * (e1 + e2 + e3);

			edge = next;
		} while (GetEdge(edge->next) != begin);
	}

	// Centroid
	B3_ASSERT(volume > B3_EPSILON);
	c /= volume;
	c += p1;
	return c;
}