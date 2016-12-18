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

#include <bounce\dynamics\shapes\capsule_shape.h>

b3CapsuleShape::b3CapsuleShape() 
{
	m_type = e_capsuleShape;
	m_radius = 0.0f;
	m_centers[0].SetZero();
	m_centers[1].SetZero();
}

b3CapsuleShape::~b3CapsuleShape() 
{
}

void b3CapsuleShape::Swap(const b3CapsuleShape& other) 
{
	m_centers[0] = other.m_centers[0];
	m_centers[1] = other.m_centers[1];
	m_radius = other.m_radius;
}

void b3CapsuleShape::ComputeMass(b3MassData* massData, float32 density) const 
{
	b3Vec3 A = m_centers[0];
	b3Vec3 B = m_centers[1];

	b3Vec3 d = B - A;
	float32 h = b3Length(d);
	B3_ASSERT(h > B3_LINEAR_SLOP);
	float32 h2 = h * h;

	float32 r = m_radius;
	float32 r2 = r * r;
	float32 r3 = r2 * r;

	// Cylinder inertia about the capsule center of mass
	b3MassData Ic_Cylinder;
	{
		// Cylinder mass
		float32 volume = B3_PI * r2 * h;
		float32 mass = density * volume;

		// Cylinder inertia about the center of mass (same as capsule center of mass)
		float32 x = (1.0f / 12.0f) * mass * (3.0f * r2 + h2);
		float32 y = 0.5f * mass * r2;
		float32 z = x;

		Ic_Cylinder.center = 0.5f * (A + B);
		Ic_Cylinder.mass = mass;
		Ic_Cylinder.I = b3Diagonal(x, y, z);
	}

	// Hemisphere inertia about the capsule center of mass
	b3MassData Ic_Hemisphere;
	{
		// Hemisphere volume and mass
		float32 volume = (2.0f / 3.0f) * B3_PI * r3;
		float32 mass = density * volume;
		
		// Ic = Io + m * d^2
		// Io = Ic - m * d^2

		// Hemisphere inertia about the origin
		float32 Io = (2.0f / 5.0f) * mass * r2;
		
		// Hemisphere center of mass relative to origin
		float32 d1 = (3.0f / 8.0f) * r;
		
		// Hemisphere inertia about the hemisphere center of mass
		float32 Ic = Io - (mass * d1 * d1);

		// Hemisphere center of mass relative to the capsule center of mass
		float32 d2 = d1 + 0.5f * h;
		
		// Hemisphere inertia about the capsule center of mass
		float32 x = Ic + (mass * d2 * d2);
		float32 y = Io;
		float32 z = x;

		Ic_Hemisphere.center.Set(0.0f, d2, 0.0f);
		Ic_Hemisphere.mass = mass;
		Ic_Hemisphere.I = b3Diagonal(x, y, z);
	}

	// Capsule inertia about the capsule center of mass
	b3MassData Ic_Capsule;
	{
		// Capsule center of mass
		Ic_Capsule.center = 0.5f * (A + B);
		// Inertia about the capsule center of mass
		// taking two hemispheres into account
		Ic_Capsule.mass = Ic_Cylinder.mass + 2.0f * Ic_Hemisphere.mass;
		Ic_Capsule.I = Ic_Cylinder.I + 2.0f * Ic_Hemisphere.I;
	}
	
	// Capsule inertia about the reference frame of the cylinder
	// Center of mass doesn't change
	B3_ASSERT(h > B3_LINEAR_SLOP);
	b3Mat33 R;
	R.y = (1.0f / h) * d;
	R.x = b3Perp(R.y);
	R.z = b3Cross(R.y, R.x);

	b3Mat33 Ic = b3RotateToFrame(Ic_Capsule.I, R);
	
	// Inertia about the center of mass
	massData->center = Ic_Capsule.center;
	massData->mass = Ic_Capsule.mass;
	massData->I = Ic;
}

void b3CapsuleShape::ComputeAABB(b3AABB3* aabb, const b3Transform& xf) const 
{
	b3Vec3 c1 = b3Mul(xf, m_centers[0]);
	b3Vec3 c2 = b3Mul(xf, m_centers[1]);
	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->m_lower = b3Min(c1, c2) - r;
	aabb->m_upper = b3Max(c1, c2) + r;
}

bool b3CapsuleShape::TestPoint(const b3Vec3& point, const b3Transform& xf) const
{
	// The point in the frame of the capsule
	b3Vec3 Q = b3MulT(xf, point);

	b3Vec3 A = m_centers[0];
	b3Vec3 B = m_centers[1];
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	float32 u = b3Dot(B - Q, AB);
	float32 v = b3Dot(Q - A, AB);

	if (v <= 0.0f)
	{
		// A
		if (b3DistanceSquared(A, Q) > m_radius * m_radius)
		{
			return false;
		}
		return true;
	}

	if (u <= 0.0f)
	{
		// B
		if (b3DistanceSquared(B, Q) > m_radius * m_radius)
		{
			return false;
		}
		return true;
	}

	// AB
	float32 s = b3Dot(AB, AB);
	B3_ASSERT(s > 0.0f);
	b3Vec3 P = (1.0f / s) * (u * A + v * B);
	if (b3DistanceSquared(P, Q) > m_radius * m_radius)
	{
		return false;
	}

	return true;
}

bool b3CapsuleShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	b3Vec3 A = input.p1;
	b3Vec3 B = input.p2;

	b3Vec3 n = B - A;
	float32 nn = b3Dot(n, n);
	
	// Check for short segment.
	if (nn < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	b3Vec3 P = b3Mul(xf, m_centers[0]);
	b3Vec3 Q = b3Mul(xf, m_centers[1]);

	b3Vec3 d = Q - P;
	float32 dd = b3Dot(d, d);
	
	// Check for short segment.
	if (dd < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	// Solve quadratic equation.
	b3Vec3 m = A - P;
	float32 nd = b3Dot(n, d);
	float32 mm = b3Dot(m, m);
	float32 md = b3Dot(m, d);
	float32 mn = b3Dot(m, n);
	
	float32 a = dd * nn - nd * nd;
	if (b3Abs(a) < 2.0f * (B3_EPSILON * B3_EPSILON))
	{
		return false;
	}

	float32 b = dd * mn - nd * md;
	float32 c = dd * (mm - m_radius * m_radius) - md * md;

	float32 disc = b * b - a * c;

	// Check for negative discriminant.
	if (disc < 0.0f)
	{
		return false;
	}

	// Find minimum intersection of the line with the cylinder.
	float32 t = -b - b3Sqrt(disc);
	t /= a;

	// Is the intersection on the segment?
	if (t < 0.0f || t > 1.0f)
	{
		return false;
	}
	
	// Hemisphere check.
	float32 tp = md + t * nd;

	b3Vec3 C;
	if (tp < 0.0f)
	{
		// P
		C = P;
	}
	else if (tp > dd)
	{
		// Q
		C = Q;
	}
	else
	{
		// PQ
		// Intersection is on the cylinder.
		b3Vec3 X = (1.0f - t) * A + t * B;
		b3Vec3 e2 = b3Cross(d, X - P);
		b3Vec3 e3 = b3Cross(e2, d);
		e3.Normalize();

		output->fraction = t;
		output->normal = e3;
		return true;
	}

	// Intersection is on hemisphere at C. 
	d = n;
	a = nn;

	m = A - C;
	b = b3Dot(m, d);
	c = b3Dot(m, m) - m_radius * m_radius;

	disc = b * b - a * c;

	// Check for negative discriminant.
	if (disc < 0.0f)
	{
		return false;
	}

	// Find the minimum time of impact of the line with the hemisphere.
	t = -b - b3Sqrt(disc);

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