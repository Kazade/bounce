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

#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/time_step.h>

b3CapsuleShape::b3CapsuleShape() 
{
	m_type = e_capsuleShape;
	m_radius = scalar(0);
}

b3CapsuleShape::~b3CapsuleShape() 
{
}

void b3CapsuleShape::Swap(const b3CapsuleShape& other) 
{
	m_vertex1 = other.m_vertex1;
	m_vertex2 = other.m_vertex2;
	m_radius = other.m_radius;
}

void b3CapsuleShape::ComputeMass(b3MassData* massData, scalar density) const 
{
	b3Vec3 A = m_vertex1;
	b3Vec3 B = m_vertex2;

	b3Vec3 d = B - A;
	
	scalar h = b3Length(d);
	B3_ASSERT(h > B3_LINEAR_SLOP);
	scalar h2 = h * h;

	scalar r = m_radius;
	scalar r2 = r * r;
	scalar r3 = r2 * r;

	//
	b3Vec3 center = scalar(0.5) * (A + B);
	scalar mass = scalar(0);
	b3Mat33 I; I.SetZero();
	
	b3Mat33 rotation;
	rotation.y = (scalar(1) / h) * d;
	rotation.x = b3Perp(rotation.y);
	rotation.z = b3Cross(rotation.y, rotation.x);

	// Cylinder
	{
		// Mass
		scalar cylinderVolume = B3_PI * r2 * h;
		scalar cylinderMass = density * cylinderVolume;

		// Inertia about the center of mass
		scalar Ixx = (scalar(1) / scalar(12)) * cylinderMass * (scalar(3) * r2 + h2);
		scalar Iyy = scalar(0.5) * cylinderMass * r2;
		// Izz = Ixx
		b3Mat33 cylinderI = b3Diagonal(Ixx, Iyy, Ixx);

		// Align the inertia with the body frame
		cylinderI = b3RotateToFrame(cylinderI, rotation);

		// Shift the inertia to the body origin
		cylinderI += cylinderMass * b3Steiner(center);
		
		// Contribute
		mass += cylinderMass;
		I += cylinderI;
	}

	// Hemispheres
	{
		// Mass
		scalar hemiVolume = (scalar(2) / scalar(3)) * B3_PI * r3;
		scalar hemiMass = density * hemiVolume;
		
		// Hemisphere inertia about the origin
		scalar Io = (scalar(2) / scalar(5)) * hemiMass * r2;

		// Hemisphere center of mass relative to the origin
		scalar coy = (scalar(3) / scalar(8)) * r;
		
		// Hemisphere inertia about the hemisphere/capsule center of mass
		scalar Iyy = Io - hemiMass * coy * coy;

		// Hemisphere center of mass relative to the capsule center of mass
		scalar ccy = coy + scalar(0.5) * h;
		
		// Hemisphere inertia about the capsule the center of mass
		scalar Ixx = Io + hemiMass * ccy * ccy;

		// Izz = Ixx 
		b3Mat33 hemiI = b3Diagonal(Ixx, Iyy, Ixx);
		
		// Align the inertia with the body frame
		hemiI = b3RotateToFrame(hemiI, rotation);

		// Shift the inertia to the body origin
		hemiI += hemiMass * b3Steiner(center);

		// Contribute twice
		mass += scalar(2) * hemiMass;
		I += scalar(2) * hemiI;
	}

	// Centroid, total mass, inertia at the origin
	massData->center = center;
	massData->mass = mass;
	massData->I = I;
}

void b3CapsuleShape::ComputeAABB(b3AABB* aabb, const b3Transform& xf) const 
{
	b3Vec3 c1 = b3Mul(xf, m_vertex1);
	b3Vec3 c2 = b3Mul(xf, m_vertex2);
	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->lowerBound = b3Min(c1, c2) - r;
	aabb->upperBound = b3Max(c1, c2) + r;
}

bool b3CapsuleShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	// The point in the frame of the capsule
	b3Vec3 Q = b3MulT(xf, sphere.vertex);

	b3Vec3 A = m_vertex1;
	b3Vec3 B = m_vertex2;
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	scalar radius = m_radius + sphere.radius;

	if (v <= scalar(0))
	{
		// A
		if (b3DistanceSquared(A, Q) > radius * radius)
		{
			return false;
		}
		return true;
	}

	if (u <= scalar(0))
	{
		// B
		if (b3DistanceSquared(B, Q) > radius * radius)
		{
			return false;
		}
		return true;
	}

	// AB
	scalar s = b3Dot(AB, AB);
	B3_ASSERT(s > scalar(0));
	b3Vec3 P = (scalar(1) / s) * (u * A + v * B);
	if (b3DistanceSquared(P, Q) > radius * radius)
	{
		return false;
	}

	return true;
}

bool b3CapsuleShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	b3Vec3 Q = sphere.vertex;

	b3Vec3 A = b3Mul(xf, m_vertex1);
	b3Vec3 B = b3Mul(xf, m_vertex2);
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	scalar radius = m_radius + sphere.radius;

	if (v <= scalar(0))
	{
		// A
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		output->point = P;
		output->normal = n;

		return true;
	}

	if (u <= scalar(0))
	{
		// B
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		output->point = P;
		output->normal = n;

		return true;
	}

	// AB
	scalar s = b3Dot(AB, AB);
	//B3_ASSERT(s > scalar(0));
	b3Vec3 P = (u * A + v * B) / s;

	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return false;
	}

	b3Vec3 AQ = Q - A;
	b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
	b3Vec3 n = b3Cross(AB_x_AQ, AB);
	if (b3Dot(n, AQ) < scalar(0))
	{
		n = -n;
	}
	n.Normalize();

	output->point = P;
	output->normal = n;
	return true;
}

bool b3CapsuleShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	b3Vec3 A = input.p1;
	b3Vec3 B = input.p2;

	b3Vec3 n = B - A;
	scalar nn = b3Dot(n, n);
	
	// Check for short segment.
	if (nn < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	b3Vec3 P = b3Mul(xf, m_vertex1);
	b3Vec3 Q = b3Mul(xf, m_vertex2);

	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	
	// Check for short segment.
	if (dd < B3_EPSILON * B3_EPSILON)
	{
		scalar a = nn;
		
		b3Vec3 m = A - P;
		scalar b = b3Dot(m, n);
		scalar c = b3Dot(m, m) - m_radius * m_radius;

		scalar disc = b * b - a * c;

		// Check for negative discriminant.
		if (disc < scalar(0))
		{
			return false;
		}

		// Find the minimum time of impact of the line with the sphere.
		scalar t = -b - b3Sqrt(disc);

		// Is the intersection point on the segment?
		if (t > scalar(0) && t <= input.maxFraction * a)
		{
			// Finish solution.
			t /= a;
			output->fraction = t;
			output->normal = b3Normalize(m + t * n);
			return true;
		}

		return false;
	}

	// Solve quadratic equation.
	b3Vec3 m = A - P;
	scalar nd = b3Dot(n, d);
	scalar mm = b3Dot(m, m);
	scalar md = b3Dot(m, d);
	scalar mn = b3Dot(m, n);
	
	scalar a = dd * nn - nd * nd;
	if (b3Abs(a) < scalar(2) * (B3_EPSILON * B3_EPSILON))
	{
		return false;
	}

	scalar b = dd * mn - nd * md;
	scalar c = dd * (mm - m_radius * m_radius) - md * md;

	scalar disc = b * b - a * c;

	// Check for negative discriminant.
	if (disc < scalar(0))
	{
		return false;
	}

	// Find minimum intersection of the line with the cylinder.
	scalar t = -b - b3Sqrt(disc);
	t /= a;

	// Is the intersection on the segment?
	if (t < scalar(0) || t > scalar(1))
	{
		return false;
	}
	
	// Hemisphere check.
	scalar tp = md + t * nd;

	b3Vec3 C;
	if (tp < scalar(0))
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
		b3Vec3 X = (scalar(1) - t) * A + t * B;
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
	if (disc < scalar(0))
	{
		return false;
	}

	// Find the minimum time of impact of the line with the hemisphere.
	t = -b - b3Sqrt(disc);

	// Is the intersection point on the segment?
	if (t > scalar(0) && t <= input.maxFraction * a)
	{
		// Finish solution.
		t /= a;
		output->fraction = t;
		output->normal = b3Normalize(m + t * d);
		return true;
	}

	return false;
}