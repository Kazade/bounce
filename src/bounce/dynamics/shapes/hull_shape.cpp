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

static B3_FORCE_INLINE void b3Subexpressions(float32& w0, float32& w1, float32& w2,
	float32& f1, float32& f2, float32& f3,
	float32& g0, float32& g1, float32& g2)
{
	float32 t0 = w0 + w1;
	float32 t1 = w0 * w0;
	float32 t2 = t1 + w1 * t0;

	f1 = t0 + w2;
	f2 = t2 + w2 * f1;
	f3 = w0 * t1 + w1 * t2 + w2 * f2;

	g0 = f2 + w0 * (f1 + w0);
	g1 = f2 + w1 * (f1 + w1);
	g2 = f2 + w2 * (f1 + w2);
}

// For explanation, see Polyhedral Mass Properties - David Eberly	
void b3HullShape::ComputeMass(b3MassData* data, float32 density) const
{
	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv24 = 1.0f / 24.0f;
	const float32 inv60 = 1.0f / 60.0f;
	const float32 inv120 = 1.0f / 120.0f;

	const float32 ks[10] = { inv6, inv24, inv24, inv24, inv60, inv60, inv60, inv120, inv120, inv120 };
	float32 is[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	for (u32 i = 0; i < m_hull->faceCount; ++i)
	{
		const b3Face* face = m_hull->GetFace(i);
		const b3HalfEdge* begin = m_hull->GetEdge(face->edge);

		const b3HalfEdge* edge = m_hull->GetEdge(begin->next);
		do
		{
			const b3HalfEdge* next = m_hull->GetEdge(edge->next);
			
			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			u32 i3 = next->origin;

			b3Vec3 p1 = m_hull->GetVertex(i1);
			b3Vec3 p2 = m_hull->GetVertex(i2);
			b3Vec3 p3 = m_hull->GetVertex(i3);

			b3Vec3 e1 = p2 - p1;
			b3Vec3 e2 = p3 - p1;

			b3Vec3 d = b3Cross(e1, e2);

			float32 f1x, f1y, f1z;
			float32 f2x, f2y, f2z;
			float32 f3x, f3y, f3z;

			float32 g0x, g0y, g0z;
			float32 g1x, g1y, g1z;
			float32 g2x, g2y, g2z;

			b3Subexpressions(p1.x, p2.x, p3.x, f1x, f2x, f3x, g0x, g1x, g2x);
			b3Subexpressions(p1.y, p2.y, p3.y, f1y, f2y, f3y, g0y, g1y, g2y);
			b3Subexpressions(p1.z, p2.z, p3.z, f1z, f2z, f3z, g0z, g1z, g2z);

			is[0] += ks[0] * (d.x * f1x);

			is[1] += ks[1] * (d.x * f2x);
			is[2] += ks[2] * (d.y * f2y);
			is[3] += ks[3] * (d.z * f2z);

			is[4] += ks[4] * (d.x * f3x);
			is[5] += ks[5] * (d.y * f3y);
			is[6] += ks[6] * (d.z * f3z);

			is[7] += ks[7] * (d.x * (p1.y * g0x + p2.y * g1x + p3.y * g2x));
			is[8] += ks[8] * (d.y * (p1.z * g0y + p2.z * g1y + p3.z * g2y));
			is[9] += ks[9] * (d.z * (p1.x * g0z + p2.x * g1z + p3.x * g2z));
			
			edge = next;
		} while (m_hull->GetEdge(edge->next) != begin);
	}

	// Apply constants
	//for (u32 i = 0; i < 10; ++i)
	//{
	//is[i] *= ks[i];
	//}

	// Volume
	float32 V = is[0];
	B3_ASSERT(V > B3_EPSILON);

	// Center of volume
	b3Vec3 c(is[1], is[2], is[3]);
	c /= V;

	// Inertia about the local origin
	b3Mat33 I;
	I.x.x = is[5] + is[6];
	I.x.y = -is[7];
	I.x.z = -is[9];

	I.y.x = I.x.y;
	I.y.y = is[4] + is[6];
	I.y.z = -is[8];

	I.z.x = I.x.z;
	I.z.y = I.y.z;
	I.z.z = is[4] + is[5];

	// Apply density
	data->center = c;
	data->mass = density * V;
	data->I = density * I;
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
