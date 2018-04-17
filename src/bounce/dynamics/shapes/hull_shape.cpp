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

void b3HullShape::ComputeMass(b3MassData* data, float32 density) const
{
	// Polyhedron mass, center of mass, and inertia.
	// Let rho be the polyhedron density per unit volume

	// mass = rho * int(1 * dV) 

	// centroid.x = (1 / mass) * rho * int(x * dV)
	// centroid.y = (1 / mass) * rho * int(y * dV)
	// centroid.z = (1 / mass) * rho * int(z * dV)

	// Ixx = rho * int((y^2 + z^2) * dV) 
	// Iyy = rho * int((x^2 + z^2) * dV)
	// Izz = rho * int((x^2 + y^2) * dV)
	
	// Ixy = -rho * int((x * y) * dV)
	// Ixz = -rho * int((x * z) * dV)
	// Iyz = -rho * int((y * z) * dV)

	// Iyx = Ixy
	// Izx = Ixz
	// Izy = Iyz
	
	// Using the Divergence's Theorem we can convert these volume integrals to surface integrals.
	// int(div(F) * dV) = int(dot(F, N) * dS) 
	// The left side is an integral over the volume V. 
	// The right side is an integral over the closed surface S of V.
	// N is the exterior normal of V along S.
	// In order to compute the surface integral we need to choose an F 
	// such that div(F) equals the function to be integrated over V.
	
	// Below are some simple choices for all F.

	// div(x, 0, 0) = 1

	// div(x^2, 0, 0) = x
	// div(0, y^2, 0) = y
	// div(0, 0, z^2) = z

	// div(x^3 / 3, 0, 0) = x^2 
	// div(0, y^3 / 3, 0) = y^2
	// div(0, 0, z^3 / 3) = z^2

	// div(x^2 * y / 2, 0, 0) = x * y  
	// div(0, y^2 * z / 2, 0) = y * z
	// div(0, 0, z^2 * x / 2) = x * z 

	// Thus, where the boundary representation is simply a set of n triangles, 
	// we can compute these integrals by summing all the integrals for each triangle 
	// of the polyhedron.
	// int(f(x, y, z) * dV) = sum(int(dot(F, N_k) * dS)), k..n.
	// If the normal N_k is constant over the triangle and s is an axis in the direction of F, 
	// we can bring N_k outside the integral
	// int(f(x, y, z) * dV) = sum(dot(N_k, s) * int(g(x, y, z) * dS)), k..n.
	
	// We need to compute surface integrals, where the g above is to be integrated along a triangle.
	// Changing coordinates from (x, y, z) to (u, v) a formula for a integral along the triangle is
	// int(g(x(u, v), y(u, v), z(u, v)) * norm(cross(e1, e2)) * du * dv)
	// where x, y, and z are given from a parametrization for a triangle
	// x = x1 + e1x * u + e2x * v 
	// y = y1 + e1y * u + e2y * v 
	// z = z1 + e1z * u + e2z * v 
	// and 0 <= u, 0 <= v, u + v <= 1
	// We can view the surface integral above also as 
	// int(g * det(D) * du * dv)
	// where D is the Jacobian of the parametrization:
	// D = cross(e1, e2)
	// We integrate g over [0, 1 - v] and then over [0, 1].

	// Thus, using the fact that 
	// N_k = D / norm(D),
	// the volume integral can be further simplified to
	// sum(dot(D, s) * int(g(x(u, v), y(u, v), z(u, v)) * du * dv))

	// These double integrals are done either by a CAS or by hand.
	// Here, it was used the great SymPy.
	// SymPy was available at http://live.sympy.org/

	b3Vec3 center;
	center.SetZero();

	float32 volume = 0.0f;

	b3Mat33 I;
	I.SetZero();

	const float32 inv3 = 1.0f / 3.0f;
	const float32 inv6 = 1.0f / 6.0f;
	const float32 inv12 = 1.0f / 12.0f;
	const float32 inv20 = 1.0f / 20.0f;
	const float32 inv60 = 1.0f / 60.0f;

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

			b3Vec3 e1 = m_hull->GetVertex(i1);
			b3Vec3 e2 = m_hull->GetVertex(i2);
			b3Vec3 e3 = m_hull->GetVertex(i3);

			float32 ex1 = e1.x, ey1 = e1.y, ez1 = e1.z;
			float32 ex2 = e2.x, ey2 = e2.y, ez2 = e2.z;
			float32 ex3 = e3.x, ey3 = e3.y, ez3 = e3.z;

			float32 a1 = ex2 - ex1, a2 = ey2 - ey1, a3 = ez2 - ez1;
			float32 b1 = ex3 - ex1, b2 = ey3 - ey1, b3 = ez3 - ez1;
			
			// D = cross(e2 - e1, e3 - e1);
			float32 D1 = a2 * b3 - a3 * b2;
			float32 D2 = a3 * b1 - a1 * b3;
			float32 D3 = a1 * b2 - a2 * b1;

			//
			float32 intx = ex1 + ex2 + ex3;
			volume += (inv6 * D1) * intx;

			//
			float32 intx2 = ex1 * ex1 + ex1 * ex2 + ex1 * ex3 + ex2 * ex2 + ex2 * ex3 + ex3 * ex3;
			float32 inty2 = ey1 * ey1 + ey1 * ey2 + ey1 * ey3 + ey2 * ey2 + ey2 * ey3 + ey3 * ey3;
			float32 intz2 = ez1 * ez1 + ez1 * ez2 + ez1 * ez3 + ez2 * ez2 + ez2 * ez3 + ez3 * ez3;

			center.x += (0.5f * inv12 * D1) * intx2;
			center.y += (0.5f * inv12 * D2) * inty2;
			center.z += (0.5f * inv12 * D3) * intz2;

			//
			float32 intx3 =
				ex1 * ex1 * ex1 +
				ex1 * ex1 * ex2 +
				ex1 * ex1 * ex3 +
				ex1 * ex2 * ex2 +
				ex1 * ex2 * ex3 +
				ex1 * ex3 * ex3 +
				ex2 * ex2 * ex2 +
				ex2 * ex2 * ex3 +
				ex2 * ex3 * ex3 +
				ex3 * ex3 * ex3;

			float32 inty3 =
				ey1 * ey1 * ey1 +
				ey1 * ey1 * ey2 +
				ey1 * ey1 * ey3 +
				ey1 * ey2 * ey2 +
				ey1 * ey2 * ey3 +
				ey1 * ey3 * ey3 +
				ey2 * ey2 * ey2 +
				ey2 * ey2 * ey3 +
				ey2 * ey3 * ey3 +
				ey3 * ey3 * ey3;
			
			float32 intz3 =
				ez1 * ez1 * ez1 +
				ez1 * ez1 * ez2 +
				ez1 * ez1 * ez3 +
				ez1 * ez2 * ez2 +
				ez1 * ez2 * ez3 +
				ez1 * ez3 * ez3 +
				ez2 * ez2 * ez2 +
				ez2 * ez2 * ez3 +
				ez2 * ez3 * ez3 +
				ez3 * ez3 * ez3;
			
			// Apply constants
			intx3 *= inv3 * inv20 * D1;
			inty3 *= inv3 * inv20 * D2;
			intz3 *= inv3 * inv20 * D3;

			I.x.x += inty3 + intz3;
			I.y.y += intx3 + intz3;	
			I.z.z += intx3 + inty3;

			// 
			float32 intx2y = 
				3.0f * ex1 * ex1 * ey1 + 
				ex1 * ex1 * ey2 +
				ex1 * ex1 * ey3 + 
				2.0f * ex1 * ex2 * ey1 +
				2.0f * ex1 * ex2 * ey2 + 
				ex1 * ex2 * ey3 + 
				2.0f * ex1 * ex3 * ey1 +
				ex1 * ex3 * ey2 + 
				2.0f * ex1 * ex3 * ey3 + 
				ex2 * ex2 * ey1 + 
				3.0f * ex2 * ex2 * ey2 + 
				ex2 * ex2 * ey3 + 
				ex2 * ex3 * ey1 + 
				2.0f * ex2 * ex3 * ey2 + 
				2.0f * ex2 * ex3 * ey3 + 
				ex3 * ex3 * ey1 + 
				ex3 * ex3 * ey2 + 
				3.0f * ex3 * ex3 * ey3;

			float32 inty2z =
				3.0f * ey1 * ey1 * ez1 +
				ey1 * ey1 * ez2 +
				ey1 * ey1 * ez3 +
				2.0f * ey1 * ey2 * ez1 +
				2.0f * ey1 * ey2 * ez2 +
				ey1 * ey2 * ez3 +
				2.0f * ey1 * ey3 * ez1 +
				ey1 * ey3 * ez2 +
				2.0f * ey1 * ey3 * ez3 +
				ey2 * ey2 * ez1 +
				3.0f * ey2 * ey2 * ez2 +
				ey2 * ey2 * ez3 +
				ey2 * ey3 * ez1 +
				2.0f * ey2 * ey3 * ez2 +
				2.0f * ey2 * ey3 * ez3 +
				ey3 * ey3 * ez1 +
				ey3 * ey3 * ez2 +
				3.0f * ey3 * ey3 * ez3;

			float32 intz2x = 
				3.0f * ez1 * ez1 * ex1 +
				ez1 * ez1 * ex2 +
				ez1 * ez1 * ex3 +
				2.0f * ez1 * ez2 * ex1 +
				2.0f * ez1 * ez2 * ex2 +
				ez1 * ez2 * ex3 +
				2.0f * ez1 * ez3 * ex1 +
				ez1 * ez3 * ex2 +
				2.0f * ez1 * ez3 * ex3 +
				ez2 * ez2 * ex1 +
				3.0f * ez2 * ez2 * ex2 +
				ez2 * ez2 * ex3 +
				ez2 * ez3 * ex1 +
				2.0f * ez2 * ez3 * ex2 +
				2.0f * ez2 * ez3 * ex3 +
				ez3 * ez3 * ex1 +
				ez3 * ez3 * ex2 +
				3.0f * ez3 * ez3 * ex3;

			// Apply constants
			intx2y *= 0.5f * inv60 * D1;
			inty2z *= 0.5f * inv60 * D2;
			intz2x *= 0.5f * inv60 * D3;

			I.x.y += intx2y;
			I.y.z += inty2z;
			I.z.x += intz2x;

			edge = next;
		} while (m_hull->GetEdge(edge->next) != begin);
	}

	// Negate
	I.x.y = -I.x.y;
	I.y.z = -I.y.z;
	I.z.x = -I.z.x;

	// Use symmetry
	I.y.x = I.x.y;
	I.z.y = I.y.z;
	I.x.z = I.x.z;
	
	// Center of mass
	B3_ASSERT(volume > B3_EPSILON);
	center /= volume;

	// Apply density
	data->center = center;
	data->mass = density * volume;
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
