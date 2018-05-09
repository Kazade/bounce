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
	u32 n = 0;
	const b3HalfEdge* begin = GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{	
		B3_ASSERT(GetFace(edge->face) == face);
		++n;
		edge = edges + edge->next;
	} while (edge != begin);

	B3_ASSERT(n >= 3);
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
		B3_ASSERT(count < edgeCount);
		++count;

		const b3HalfEdge* next = edges + e->next;
		e = edges + next->twin;
	} while (e != begin);
}