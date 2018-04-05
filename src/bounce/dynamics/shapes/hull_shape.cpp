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
#include <bounce/dynamics/time_step.h>

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
	// Compute mass data
	b3Vec3 center(0.0f, 0.0f, 0.0f);
	float32 volume = 0.0f;
	b3Mat33 I;
	I.SetZero();

	const float32 inv4 = 0.25f;
	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv60 = 1.0f / 60.0f;
	const float32 inv120 = 1.0f / 120.0f;

	b3Vec3 diag(0.0f, 0.0f, 0.0f);
	b3Vec3 off_diag(0.0f, 0.0f, 0.0f);
	
	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		const b3Face* face = m_hull->GetFace(i);
		const b3HalfEdge* begin = m_hull->GetEdge(face->edge);
		
		const b3HalfEdge* edge = m_hull->GetEdge(begin->next);
		do
		{
			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			const b3HalfEdge* next = m_hull->GetEdge(edge->next);
			u32 i3 = next->origin;

			b3Vec3 p2 = m_hull->vertices[i1];
			b3Vec3 p3 = m_hull->vertices[i2];
			b3Vec3 p4 = m_hull->vertices[i3];
			
			b3Vec3 e1 = p2;
			b3Vec3 e2 = p3;
			b3Vec3 e3 = p4;
			
			float32 D = b3Det(e1, e2, e3);

			float32 tetraVolume = inv6 * D;
			volume += tetraVolume;
			
			// Volume weighted centroid
			center += tetraVolume * inv4 * (e1 + e2 + e3);

			// Volume weighted inertia tensor
			// https://github.com/melax/sandbox
			for (u32 j = 0; j < 3; ++j)
			{
				u32 j1 = (j + 1) % 3;
				u32 j2 = (j + 2) % 3;

				diag[j] += inv60 * D *
					(e1[j] * e2[j] + e2[j] * e3[j] + e3[j] * e1[j] +
					 e1[j] * e1[j] + e2[j] * e2[j] + e3[j] * e3[j]);

				off_diag[j] += inv120 * D  *
					(e1[j1] * e2[j2] + e2[j1] * e3[j2] + e3[j1] * e1[j2] +
					 e1[j1] * e3[j2] + e2[j1] * e1[j2] + e3[j1] * e2[j2] +
					 e1[j1] * e1[j2] * 2.0f + e2[j1] * e2[j2] * 2.0f + e3[j1] * e3[j2] * 2.0f);
			}
			
			edge = next;
		} while (m_hull->GetEdge(edge->next) != begin);
	}

	// Total mass
	massData->mass = density * volume;
	
	// Center of mass
	B3_ASSERT(volume > B3_EPSILON);
	float32 inv_volume = 1.0f / volume;
	center *= inv_volume;

	diag *= inv_volume;
	off_diag *= inv_volume;

	I.x.Set(diag.y + diag.z, -off_diag.z, -off_diag.y);
	I.y.Set(-off_diag.z, diag.x + diag.z, -off_diag.x);
	I.z.Set(-off_diag.y, -off_diag.x, diag.x + diag.y);

	// Center of mass 
	massData->center = center;

	// Inertia tensor relative to the local origin
	massData->I = massData->mass * I;
}

void b3HullShape::ComputeAABB(b3AABB3* aabb, const b3Transform& xf) const
{
	aabb->Compute(m_hull->vertices, m_hull->vertexCount, xf);
	aabb->Extend(m_radius);
}

bool b3HullShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3Vec3 support = b3MulT(xf, sphere.vertex);
	float32 radius = m_radius + sphere.radius;

	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		b3Plane plane = m_hull->GetPlane(i);
		float32 separation = b3Distance(support, plane);

		if (separation > radius)
		{
			return false;
		}
	}

	return true;
}

bool b3HullShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	// Perform computations in the local space of the first hull.
	b3Vec3 support = b3MulT(xf, sphere.vertex);
	float32 radius = m_radius + sphere.radius;
	
	u32 maxIndex = ~0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		b3Plane plane = m_hull->GetPlane(i);
		float32 separation = b3Distance(support, plane);

		if (separation > radius)
		{
			return false;
		}

		if (separation > maxSeparation)
		{
			maxIndex = i;
			maxSeparation = separation;
		}
	}

	if (maxIndex != ~0)
	{
		output->separation = maxSeparation;
		output->normal = b3Mul(xf.rotation, m_hull->GetPlane(maxIndex).normal);
		return true;
	}
	
	B3_ASSERT(false);
	return false;
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
