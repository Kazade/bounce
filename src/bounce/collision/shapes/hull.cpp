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

static B3_FORCE_INLINE void b3Subexpressions(float32& w0, float32& w1, float32& w2,
	float32& f1, float32& f2, float32& f3)
{
	float32 t0 = w0 + w1;
	float32 t1 = w0 * w0;
	float32 t2 = t1 + w1 * t0;

	f1 = t0 + w2;
	f2 = t2 + w2 * f1;
	f3 = w0 * t1 + w1 * t2 + w2 * f2;
}

b3Vec3 b3Hull::GetCentroid() const
{
	Validate();

	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv24 = 1.0f / 24.0f;
	
	float32 intgex = 0.0f;

	float32 intgcx = 0.0f;
	float32 intgcy = 0.0f;
	float32 intgcz = 0.0f;

	for (u32 i = 0; i < faceCount; ++i)
	{
		const b3Face* face = GetFace(i);
		const b3HalfEdge* begin = GetEdge(face->edge);

		const b3HalfEdge* edge = GetEdge(begin->next);
		do
		{
			const b3HalfEdge* next = GetEdge(edge->next);
			
			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			u32 i3 = next->origin;

			b3Vec3 p1 = GetVertex(i1);
			b3Vec3 p2 = GetVertex(i2);
			b3Vec3 p3 = GetVertex(i3);

			b3Vec3 e1 = p2 - p1;
			b3Vec3 e2 = p3 - p1;

			b3Vec3 d = b3Cross(e1, e2);

			float32 f1x, f1y, f1z;
			float32 f2x, f2y, f2z;
			float32 f3x, f3y, f3z;

			b3Subexpressions(p1.x, p2.x, p3.x, f1x, f2x, f3x);
			b3Subexpressions(p1.y, p2.y, p3.y, f1y, f2y, f3y);
			b3Subexpressions(p1.z, p2.z, p3.z, f1z, f2z, f3z);

			intgex += inv6 * (d.x * f1x);

			intgcx += inv24 * (d.x * f2x);
			intgcy += inv24 * (d.y * f2y);
			intgcz += inv24 * (d.z * f2z);

			edge = next;
		} while (GetEdge(edge->next) != begin);
	}
	
	// Apply constants
	//intgex *= inv6;

	//intgcx *= inv24;
	//intgcy *= inv24;
	//intgcz *= inv24;

	// Center of volume
	B3_ASSERT(intgex > B3_EPSILON);
	intgcx /= intgex;
	intgcy /= intgex;
	intgcz /= intgex;

	return b3Vec3(intgcx, intgcy, intgcz);
}