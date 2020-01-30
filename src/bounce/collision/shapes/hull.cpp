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
	u32 edgeIndex = (u32)(e - edges);
	
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

void b3Hull::Scale(const b3Vec3& scale)
{
	// https://irlanrobson.github.io/2019/10/01/how-to-transform-a-plane,-with-scale/
	B3_ASSERT(scale.x > scalar(0));
	B3_ASSERT(scale.y > scalar(0));
	B3_ASSERT(scale.z > scalar(0));

	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3MulCW(scale, vertices[i]);
	}

	// M = S
	// M^-1 = S^-1
	// (M^-1)^T = (S^-1)^T = S^-1
	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / scale.x;
	inv_scale.y = scalar(1) / scale.y;
	inv_scale.z = scalar(1) / scale.z;

	for (u32 i = 0; i < faceCount; ++i)
	{
		b3Plane oldPlane = planes[i];

		b3Plane newPlane;
		newPlane.normal = b3MulCW(inv_scale, oldPlane.normal);
		
		scalar len = newPlane.normal.Normalize();
		B3_ASSERT(len > scalar(0));
		
		newPlane.offset = oldPlane.offset / len;

		planes[i] = newPlane;
	}

	centroid = b3MulCW(scale, centroid);
}

void b3Hull::Rotate(const b3Quat& rotation)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(rotation, vertices[i]);
	}

	for (u32 i = 0; i < faceCount; ++i)
	{
		planes[i].normal = b3Mul(rotation, planes[i].normal);
	}
}

void b3Hull::Translate(const b3Vec3& translation)
{
	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] += translation;
	}

	for (u32 i = 0; i < faceCount; ++i)
	{
		planes[i].offset += b3Dot(planes[i].normal, translation);
	}
}

void b3Hull::Transform(const b3Transform& xf, const b3Vec3& scale)
{
	// https://irlanrobson.github.io/2019/10/01/how-to-transform-a-plane,-with-scale/
	B3_ASSERT(scale.x > scalar(0));
	B3_ASSERT(scale.y > scalar(0));
	B3_ASSERT(scale.z > scalar(0));

	for (u32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b3Mul(xf.rotation, b3MulCW(scale, vertices[i])) + xf.translation;
	}
	
	// M = R * S
	// M^-1 = S^-1 * R^-1
	// (M^-1)^T = (R^-1)^T * (S^-1)^T = R * S^-1
	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / scale.x;
	inv_scale.y = scalar(1) / scale.y;
	inv_scale.z = scalar(1) / scale.z;

	for (u32 i = 0; i < faceCount; ++i)
	{
		b3Plane oldPlane = planes[i];

		b3Plane newPlane;
		newPlane.normal = b3Mul(xf.rotation, b3MulCW(inv_scale, oldPlane.normal));
		
		scalar len = newPlane.normal.Normalize();
		B3_ASSERT(len > scalar(0));

		newPlane.offset = oldPlane.offset / len + b3Dot(newPlane.normal, xf.translation);

		planes[i] = newPlane;
	}

	centroid = b3Mul(xf.rotation, b3MulCW(scale, centroid)) + xf.translation;
}

void b3Hull::Dump() const
{
	b3Log("b3Vec3 vertices[%d]\n", vertexCount);
	b3Log("b3HalfEdge edges[%d]\n", edgeCount);
	b3Log("b3Face faces[%d]\n", faceCount);
	b3Log("b3Plane planes[%d]\n", faceCount);

	for (u32 i = 0; i < vertexCount; ++i)
	{
		b3Vec3 v = vertices[i];

		b3Log("vertices[%d] = b3Vec3(%.20f, %.20f, %.20f);\n", i, v.x, v.y, v.z);
	}

	b3Log("\n");

	for (u32 i = 0; i < edgeCount; ++i)
	{
		b3HalfEdge* he = edges + i;

		b3Log("edges[%d] = b3MakeEdge(%d, %d, %d, %d, %d);\n", i, he->origin, he->twin, he->face, he->prev, he->next);
	}

	b3Log("\n");

	for (u32 i = 0; i < faceCount; ++i)
	{
		b3Log("faces[%d].edge = %d;\n", i, faces[i].edge);
	}

	b3Log("\n");

	for (u32 i = 0; i < faceCount; ++i)
	{
		b3Plane plane = planes[i];

		b3Log("planes[%d].normal = b3Vec3(%.20f, %.20f, %.20f);\n", i, plane.normal.x, plane.normal.y, plane.normal.z);
		b3Log("planes[%d].offset = %.20f;\n", i, plane.offset);
	}

	b3Log("centroid = b3Vec3(%.20f, %.20f, %.20f);\n", centroid.x, centroid.y, centroid.z);
}
