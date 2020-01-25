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

#include <bounce/dynamics/shapes/triangle_shape.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/contacts/collide/collide.h>
#include <bounce/collision/gjk/gjk.h>

b3TriangleShape::b3TriangleShape()
{
	m_type = e_triangleShape;
	m_radius = B3_HULL_RADIUS;
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

b3TriangleShape::~b3TriangleShape()
{
}

void b3TriangleShape::Swap(const b3TriangleShape& other)
{
	m_radius = other.m_radius;
	m_vertex1 = other.m_vertex1;
	m_vertex2 = other.m_vertex2;
	m_vertex3 = other.m_vertex3;
	m_hasE1Vertex = other.m_hasE1Vertex;
	m_hasE2Vertex = other.m_hasE2Vertex;
	m_hasE3Vertex = other.m_hasE3Vertex;
	m_e1Vertex = other.m_e1Vertex;
	m_e2Vertex = other.m_e2Vertex;
	m_e3Vertex = other.m_e3Vertex;
}

void b3TriangleShape::Set(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_vertex3 = v3;
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

void b3TriangleShape::ComputeMass(b3MassData* massData, scalar density) const
{
	B3_NOT_USED(density);
	massData->center = scalar(0.5) * (m_vertex1 + m_vertex2 + m_vertex3);
	massData->mass = 0.0f;
	massData->I.SetZero();
}

void b3TriangleShape::ComputeAABB(b3AABB* aabb, const b3Transform& xf) const
{
	b3Vec3 lower = b3Min(m_vertex1, b3Min(m_vertex2, m_vertex3));
	b3Vec3 upper = b3Max(m_vertex1, b3Max(m_vertex2, m_vertex3));

	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

bool b3TriangleShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3ShapeGJKProxy proxy1;
	proxy1.vertexBuffer[0] = m_vertex1;
	proxy1.vertexBuffer[1] = m_vertex2;
	proxy1.vertexBuffer[2] = m_vertex3;
	proxy1.vertexCount = 3;
	proxy1.vertices = proxy1.vertexBuffer;

	b3ShapeGJKProxy proxy2;
	proxy2.vertexBuffer[0] = b3MulT(xf, sphere.vertex);
	proxy2.vertexCount = 1;
	proxy2.vertices = proxy2.vertexBuffer;

	b3GJKOutput query = b3GJK(b3Transform_identity, proxy1, b3Transform_identity, proxy2, false);

	if (query.distance <= m_radius + sphere.radius)
	{
		return true;
	}

	return false;
}

bool b3TriangleShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	b3ShapeGJKProxy proxy1;
	proxy1.vertexBuffer[0] = m_vertex1;
	proxy1.vertexBuffer[1] = m_vertex2;
	proxy1.vertexBuffer[2] = m_vertex3;
	proxy1.vertexCount = 3;
	proxy1.vertices = proxy1.vertexBuffer;

	b3ShapeGJKProxy proxy2;
	proxy2.vertexBuffer[0] = b3MulT(xf, sphere.vertex);
	proxy2.vertexCount = 1;
	proxy2.vertices = proxy2.vertexBuffer;

	b3GJKOutput query = b3GJK(b3Transform_identity, proxy1, b3Transform_identity, proxy2, false);

	scalar radius = m_radius + sphere.radius;

	if (query.distance <= radius)
	{
		b3Vec3 p1 = b3Mul(xf, query.point1);
		b3Vec3 p2 = b3Mul(xf, query.point2);
		scalar distance = query.distance;

		b3Vec3 n;
		if (distance > B3_EPSILON)
		{
			n = p2 - p1;
			n.Normalize();
		}
		else
		{
			n = b3Cross(m_vertex2 - m_vertex1, m_vertex3 - m_vertex1);
			n.Normalize();
			n = b3Mul(xf.rotation, n);
		}

		output->point = p1;
		output->normal = n;

		return true;
	}

	return false;
}

bool b3TriangleShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	// Put the ray into the triangle's frame of reference.
	b3Vec3 p1 = b3MulT(xf, input.p1);
	b3Vec3 p2 = b3MulT(xf, input.p2);
	
	b3RayCastInput localInput;
	localInput.p1 = p1;
	localInput.p2 = p2;
	localInput.maxFraction = input.maxFraction;

	b3RayCastOutput localOutput;
	if (b3RayCast(&localOutput, &localInput, m_vertex1, m_vertex2, m_vertex3))
	{
		output->fraction = localOutput.fraction;
		output->normal = b3Mul(xf.rotation, localOutput.normal);
		return true;
	}
	
	return false;
}