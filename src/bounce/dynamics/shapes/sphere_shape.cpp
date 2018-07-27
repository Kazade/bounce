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

#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/time_step.h>

b3SphereShape::b3SphereShape() 
{
	m_type = e_sphereShape;
	m_radius = 0.0f;
	m_center.SetZero();
}

b3SphereShape::~b3SphereShape() 
{
}

void b3SphereShape::Swap(const b3SphereShape& other) 
{
	m_center = other.m_center;
	m_radius = other.m_radius;
}

void b3SphereShape::ComputeMass(b3MassData* massData, float32 density) const 
{
	float32 volume = (4.0f / 3.0f) * B3_PI * m_radius * m_radius * m_radius;
	float32 mass = density * volume;
	
	// Inertia about the local shape center of mass 
	// Then shift to the local body origin
	float32 I = mass * (0.4f * m_radius * m_radius + b3Dot(m_center, m_center));
	
	massData->center = m_center;
	massData->mass = mass;
	massData->I = b3Diagonal(I);
}

void b3SphereShape::ComputeAABB(b3AABB3* aabb, const b3Transform& xf) const 
{
	b3Vec3 center = b3Mul(xf, m_center);
	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->m_lower = center - r;
	aabb->m_upper = center + r;
}

bool b3SphereShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3Vec3 center = b3Mul(xf, m_center);
	float32 radius = m_radius + sphere.radius;
	float32 rr = radius * radius;
	b3Vec3 d = sphere.vertex - center;
	float32 dd = b3Dot(d, d);
	return dd <= rr;
}

bool b3SphereShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	b3Vec3 center = b3Mul(xf, m_center);
	float32 radius = m_radius + sphere.radius;
	float32 rr = radius * radius;
	b3Vec3 d = sphere.vertex - center;
	float32 dd = b3Dot(d, d);
	
	if (dd <= rr)
	{
		float32 d_len = b3Sqrt(dd);

		output->point = center;
		output->separation = d_len - radius;
		output->normal.Set(0.0f, 1.0, 0.0f);
		if (d_len > B3_EPSILON)
		{
			output->normal = d / d_len;
		}

		return true;
	}

	return false;
}

bool b3SphereShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	// dot(x - c, x - c) - r^2 = 0
	// S = p1 + t * (p2 - p1)
	// Solve for t:
	// dot(p1 + t * d - c, P + t * d - c) - r^2 =
	// dot(d, d) * t^2 + 2 * dot(m, d) * t + dot(m, m) - r^2 = 0
	// m = p1 - c
	// d = p2 - p1
	b3Vec3 d = input.p2 - input.p1;
	float32 a = b3Dot(d, d);
	
	// Check for short segment.
	if (a < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}
		
	// Solve quadratic equation
	// a * t^2 + 2 * b * t + c = 0
	// a = dot(d, d)
	// b = dot(m, d)
	// c = dot(m, m) - r^2
	// t = -b +/- sqrt(b * b - a * c) / a
	b3Vec3 m = input.p1 - b3Mul(xf, m_center);
	float32 b = b3Dot(m, d);
	float32 c = b3Dot(m, m) - m_radius * m_radius;
	
	float32 disc = b * b - a * c;

	// Check for negative discriminant.
	// Does the ray misses the sphere completely?
	if (disc < 0.0f)
	{
		return false;
	}

	// Find the minimum time of impact of the line with the sphere.
	// t_min = -b - sqrt(disc)
	// t_max = -b + sqrt(disc) 
	float32 t = -b - b3Sqrt(disc);
	
	// Is the intersection point on the segment?
	if (t > 0.0f && t <= input.maxFraction * a) 
	{
		// Finish solution.
		t /= a;
		output->fraction = t;
		output->normal = b3Normalize(m + t * d);
		return true;
	}
	
	return false;
}