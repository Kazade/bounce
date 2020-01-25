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

#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/collision/shapes/mesh.h>

b3MeshShape::b3MeshShape() 
{
	m_type = e_meshShape;
	m_radius = B3_HULL_RADIUS;
	m_mesh = nullptr;
	m_scale.Set(scalar(1), scalar(1), scalar(1));
}

b3MeshShape::~b3MeshShape() 
{
}

void b3MeshShape::Swap(const b3MeshShape& other) 
{
	m_radius = other.m_radius;
	m_mesh = other.m_mesh;
	m_scale = other.m_scale;
}

void b3MeshShape::ComputeMass(b3MassData* massData, scalar density) const 
{
	B3_NOT_USED(density);	
	massData->center.SetZero();
	massData->mass = scalar(0);
	massData->I.SetZero();
}

void b3MeshShape::ComputeAABB(b3AABB* output, const b3Transform& xf) const 
{
	b3AABB aabb;
	aabb.Set(m_mesh->vertices, m_mesh->vertexCount, m_scale, xf);
	aabb.Extend(m_radius);

	*output = aabb;
}

void b3MeshShape::ComputeAABB(b3AABB* output, const b3Transform& xf, u32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	const b3MeshTriangle* triangle = m_mesh->triangles + index;
	b3Vec3 v1 = b3Mul(xf, b3MulCW(m_scale, m_mesh->vertices[triangle->v1]));
	b3Vec3 v2 = b3Mul(xf, b3MulCW(m_scale, m_mesh->vertices[triangle->v2]));
	b3Vec3 v3 = b3Mul(xf, b3MulCW(m_scale, m_mesh->vertices[triangle->v3]));
	output->SetTriangle(v1, v2, v3);
	output->Extend(m_radius);
}

bool b3MeshShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	B3_NOT_USED(sphere);
	B3_NOT_USED(xf);
	return false;
}

bool b3MeshShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf) const
{
	B3_NOT_USED(output);
	B3_NOT_USED(sphere);
	B3_NOT_USED(xf);
	return false;
}

bool b3MeshShape::TestSphere(b3TestSphereOutput* output, const b3Sphere& sphere, const b3Transform& xf, u32 index) const
{
	B3_NOT_USED(output);
	B3_NOT_USED(sphere);
	B3_NOT_USED(xf);
	B3_NOT_USED(index);
	return false;
}

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf, u32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	b3MeshTriangle* triangle = m_mesh->triangles + index;
	
	b3Vec3 v1 = b3MulCW(m_scale, m_mesh->vertices[triangle->v1]);
	b3Vec3 v2 = b3MulCW(m_scale, m_mesh->vertices[triangle->v2]);
	b3Vec3 v3 = b3MulCW(m_scale, m_mesh->vertices[triangle->v3]);

	// Put the ray into the mesh's frame of reference.
	b3Vec3 p1 = b3MulT(xf, input.p1);
	b3Vec3 p2 = b3MulT(xf, input.p2);
	
	b3RayCastInput subInput;
	subInput.p1 = p1;
	subInput.p2 = p2;
	subInput.maxFraction = input.maxFraction;

	b3RayCastOutput subOutput;
	if (b3RayCast(&subOutput, &subInput, v1, v2, v3))
	{
		output->fraction = subOutput.fraction;
		output->normal = b3Mul(xf.rotation, subOutput.normal);
		return true;
	}

	return false;
}

struct b3MeshShapeRayCastCallback
{
	scalar Report(const b3RayCastInput& subInput, u32 proxyId)
	{
		B3_NOT_USED(subInput);

		u32 childIndex = mesh->m_mesh->tree.GetUserData(proxyId);
		
		b3RayCastOutput childOutput;
		if (mesh->RayCast(&childOutput, input, xf, childIndex))
		{
			// Track minimum time of impact to require less memory.
			if (childOutput.fraction < output.fraction)
			{
				hit = true;
				output = childOutput;
			}
		}
		
		return scalar(1);
	}

	b3RayCastInput input;
	const b3MeshShape* mesh;
	b3Transform xf;
	
	bool hit;
	b3RayCastOutput output;
};

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	b3MeshShapeRayCastCallback callback;
	callback.input = input;
	callback.mesh = this;
	callback.xf = xf;
	callback.hit = false;
	callback.output.fraction = B3_MAX_SCALAR;
	
	B3_ASSERT(m_scale.x != scalar(0));
	B3_ASSERT(m_scale.y != scalar(0));
	B3_ASSERT(m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / m_scale.x;
	inv_scale.y = scalar(1) / m_scale.y;
	inv_scale.z = scalar(1) / m_scale.z;

	b3RayCastInput treeInput;
	treeInput.p1 = b3MulCW(inv_scale, b3MulT(xf, input.p1));
	treeInput.p2 = b3MulCW(inv_scale, b3MulT(xf, input.p2));
	treeInput.maxFraction = input.maxFraction;
	m_mesh->tree.RayCast(&callback, treeInput);

	output->fraction = callback.output.fraction;
	output->normal = callback.output.normal;

	return callback.hit;
}