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

#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/common/template/array.h>

b3HullShape::b3HullShape()
{
	m_type = e_hullShape;
	m_radius = B3_HULL_RADIUS;
	m_hull = NULL;
}

b3HullShape::~b3HullShape()
{
}

void b3HullShape::Swap(const b3HullShape& other)
{
	m_radius = other.m_radius;
	m_hull = other.m_hull;
}

void b3HullShape::ComputeMass(b3MassData* massData, float32 density) const
{
	// Build triangles for hull
	b3StackArray<b3Triangle, 256> triangles;
	
	// Use a small buffer for polygons.
	u32 polygon[B3_MAX_HULL_FEATURES];
	u32 vCount = 0;

	// Convert polygons to triangles
	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		// Build convex polygon for loop
		const b3Face* face = m_hull->GetFace(i);
		const b3HalfEdge* begin = m_hull->GetEdge(face->edge);
		const b3HalfEdge* edge = begin;
		do
		{
			polygon[vCount++] = u32(edge->origin);
			edge = m_hull->GetEdge(edge->next);
		} while (edge != begin);
		
		// Triangulate convex polygon
		B3_ASSERT(vCount > 2);
		for (u32 j = 1; j < vCount - 1; ++j)
		{
			b3Triangle triangle;
			triangle.v1 = polygon[0];
			triangle.v2 = polygon[j];
			triangle.v3 = polygon[j + 1];
			triangles.PushBack(triangle);
		}

		vCount = 0;
	}
	vCount = 0;

	// Compute mass data
	b3Vec3 center(0.0f, 0.0f, 0.0f);
	float32 volume = 0.0f;
	b3Mat33 I;
	I.SetZero();

	// Pick reference point not too away from the origin 
	// to minimize floating point rounding errors.
	b3Vec3 v1(0.0f, 0.0f, 0.0f);

	const float32 inv4 = 0.25f;
	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv60 = 1.0f / 60.0f;
	const float32 inv120 = 1.0f / 120.0f;

	b3Vec3 diag(0.0f, 0.0f, 0.0f);
	b3Vec3 offDiag(0.0f, 0.0f, 0.0f);

	for (u32 i = 0; i < triangles.Count(); ++i)
	{
		const b3Triangle* triangle = triangles.Get(i);

		b3Vec3 v2 = m_hull->GetVertex(triangle->v1);
		b3Vec3 v3 = m_hull->GetVertex(triangle->v2);
		b3Vec3 v4 = m_hull->GetVertex(triangle->v3);
		b3Vec3 tetraCenter = inv4 * (v1 + v2 + v3 + v4);

		b3Vec3 e1 = v2 - v1;
		b3Vec3 e2 = v3 - v1;
		b3Vec3 e3 = v4 - v1;
		float32 det = b3Det(e1, e2, e3);
		float32 tetraVolume = inv6 * det;

		// Volume weighted center of mass
		center += tetraVolume * tetraCenter;
		volume += tetraVolume;

		// Volume weighted inertia tensor
		// https://github.com/melax/sandbox
		for (u32 j = 0; j < 3; ++j)
		{
			u32 j1 = (j + 1) % 3;
			u32 j2 = (j + 2) % 3;

			diag[j] += inv60 * det *
				(e1[j] * e2[j] + e2[j] * e3[j] + e3[j] * e1[j] +
				 e1[j] * e1[j] + e2[j] * e2[j] + e3[j] * e3[j]);

			offDiag[j] += inv120 * det  *
				(e1[j1] * e2[j2] + e2[j1] * e3[j2] + e3[j1] * e1[j2] +
				 e1[j1] * e3[j2] + e2[j1] * e1[j2] + e3[j1] * e2[j2] +
				 e1[j1] * e1[j2] * 2.0f + e2[j1] * e2[j2] * 2.0f + e3[j1] * e3[j2] * 2.0f);
		}
	}

	B3_ASSERT(volume > 0.0f);
	float32 invVolume = 0.0f;
	if (volume != 0.0f)
	{
		invVolume = 1.0f / volume;
	}
	
	diag = invVolume * diag;
	offDiag = invVolume * offDiag;

	I.x.Set(diag.y + diag.z, -offDiag.z, -offDiag.y);
	I.y.Set(-offDiag.z, diag.x + diag.z, -offDiag.x);
	I.z.Set(-offDiag.y, -offDiag.x, diag.x + diag.y);

	massData->center = invVolume * center;
	massData->mass = density * volume;
	massData->I = massData->mass * I;
}

void b3HullShape::ComputeAABB(b3AABB3* aabb, const b3Transform& xf) const
{
	aabb->Compute(m_hull->vertices, m_hull->vertexCount, xf);
	aabb->Extend(m_radius);
}

bool b3HullShape::TestPoint(const b3Vec3& point, const b3Transform& xf) const
{
	// Put the point into the hull's frame of reference.
	b3Vec3 p = b3MulT(xf, point);
	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		float32 d = b3Distance(p, m_hull->planes[i]);
		if (d > m_radius)
		{
			return false;
		}
	}
	return true;
}

bool b3HullShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	u32 planeCount = m_hull->faceCount;
	const b3Plane* planes = m_hull->planes;

	// Put the segment into the poly's frame of reference.
	b3Vec3 p1 = b3MulT(xf.rotation, input.p1 - xf.position);
	b3Vec3 p2 = b3MulT(xf.rotation, input.p2 - xf.position);
	b3Vec3 d = p2 - p1;

	float32 lower = 0.0f;
	float32 upper = input.maxFraction;

	i32 index = -1;

	// s(lower) = p1 + lower * d, 0 <= lower <= kupper
	// The segment intersects the plane if a 'lower' exists
	// for which s(lower) is inside all half-spaces.

	// Solve line segment to plane:
	// dot(n, s(lower)) = offset
	// dot(n, p1 + lower * d) = offset
	// dot(n, p1) + dot(n, lower * d) = offset
	// dot(n, p1) + lower * dot(n, d) = offset
	// lower * dot(n, d) = offset - dot(n, p1)
	// lower = (offset - dot(n, p1)) / dot(n, d)

	for (u32 i = 0; i < planeCount; ++i)
	{
		float32 numerator = planes[i].offset - b3Dot(planes[i].normal, p1);
		float32 denominator = b3Dot(planes[i].normal, d);

		if (denominator == 0.0f)
		{
			// s is parallel to this half-space.
			if (numerator < 0.0f)
			{
				// s is outside of this half-space.
				// dot(n, p1) and dot(n, p2) < 0.
				return false;
			}
		}
		else
		{
			// Original predicates:
			// lower < numerator / denominator, for denominator < 0
			// upper < numerator / denominator, for denominator < 0
			// Optimized predicates:
			// lower * denominator > numerator
			// upper * denominator > numerator
			if (denominator < 0.0f)
			{
				// s enters this half-space.
				if (numerator < lower * denominator)
				{
					// Increase lower.
					lower = numerator / denominator;
					index = i;
				}
			}
			else
			{
				// s exits the half-space.	
				if (numerator < upper * denominator)
				{
					// Decrease upper.
					upper = numerator / denominator;
				}
			}

			// Exit if intersection becomes empty.
			if (upper < lower)
			{
				return false;
			}
		}
	}

	B3_ASSERT(lower >= 0.0f && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		output->normal = b3Mul(xf.rotation, planes[index].normal);
		return true;
	}

	return false;
}
