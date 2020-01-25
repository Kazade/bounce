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

#include <bounce/dynamics/shapes/sdf_shape.h>
#include <bounce/collision/shapes/sdf.h>

b3SDFShape::b3SDFShape()
{
	m_type = e_sdfShape;
	m_radius = B3_HULL_RADIUS;
	m_sdf = nullptr;
	m_scale.Set(scalar(1), scalar(1), scalar(1));
}

b3SDFShape::~b3SDFShape()
{
}

void b3SDFShape::Swap(const b3SDFShape& other)
{
	m_radius = other.m_radius;
	m_sdf = other.m_sdf;
	m_scale = other.m_scale;
}

void b3SDFShape::ComputeMass(b3MassData* massData, scalar density) const
{
	B3_NOT_USED(density);
	massData->center.SetZero();
	massData->mass = scalar(0);
	massData->I.SetZero();
}

void b3SDFShape::ComputeAABB(b3AABB* output, const b3Transform& xf) const
{
	b3AABB aabb = b3TransformAABB(m_sdf->GetDomain(), m_scale, xf);
	aabb.Extend(m_radius);
	
	*output = aabb;
}

bool b3SDFShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	B3_ASSERT(m_scale.x != scalar(0));
	B3_ASSERT(m_scale.y != scalar(0));
	B3_ASSERT(m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / m_scale.x;
	inv_scale.y = scalar(1) / m_scale.y;
	inv_scale.z = scalar(1) / m_scale.z;
	
	b3Vec3 vertex = b3MulCW(inv_scale, b3MulT(xf, sphere.vertex));
	scalar radius = m_radius + sphere.radius;

	double distance;
	bool ok = m_sdf->Evaluate(vertex, distance);
	if (ok)
	{
		return distance <= radius;
	}

	return false;
}

bool b3SDFShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	B3_ASSERT(m_scale.x != scalar(0));
	B3_ASSERT(m_scale.y != scalar(0));
	B3_ASSERT(m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / m_scale.x;
	inv_scale.y = scalar(1) / m_scale.y;
	inv_scale.z = scalar(1) / m_scale.z;

	b3Vec3 vertex = b3MulCW(inv_scale, b3MulT(xf, sphere.vertex));
	scalar radius = m_radius + sphere.radius;

	// M = R * S
	// M^-1 = S^-1 * R^-1
	// (M^-1)^T = (R^-1)^T * (S^-1)^T = R * S^-1

	double distance;
	b3Vec3 normal;
	bool ok = m_sdf->Evaluate(vertex, distance, &normal);
	if (ok)
	{
		if (distance <= radius)
		{
			normal.Normalize();

			output->normal = b3Mul(xf.rotation, b3MulCW(inv_scale, normal));
			output->normal.Normalize();
			output->point = sphere.vertex - distance * output->normal;

			return true;
		}
	}
	
	return false;
}

bool b3SDFShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	return false;
}
