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

#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/collision/shapes/mesh.h>

b3MeshShape::b3MeshShape() 
{
	m_type = e_meshShape;
	m_radius = B3_HULL_RADIUS;
	m_mesh = NULL;
}

b3MeshShape::~b3MeshShape() 
{
}

void b3MeshShape::Swap(const b3MeshShape& other) 
{
	m_radius = other.m_radius;
	m_mesh = other.m_mesh;
}

void b3MeshShape::ComputeMass(b3MassData* massData, float32 density) const 
{
	B3_NOT_USED(density);
	u32 n = m_mesh->vertexCount;
	b3Vec3 c(0.0f, 0.0f, 0.0f);
	for (u32 i = 0; i < n; ++i)
	{
		c += m_mesh->vertices[i];
	}
	if (n > 0)
	{
		c = 1.0f / float32(n) * c;
	}	
	massData->center = c;
	massData->mass = 0.0f;
	massData->I.SetZero();
}

void b3MeshShape::ComputeAABB(b3AABB3* output, const b3Transform& xf) const 
{
	output->Compute(m_mesh->vertices, m_mesh->vertexCount, xf);
	output->Extend(m_radius);
}

void b3MeshShape::ComputeAABB(b3AABB3* output, const b3Transform& xf, u32 index) const
{
	*output = m_mesh->GetTriangleAABB(index);
}

bool b3MeshShape::TestPoint(const b3Vec3& point, const b3Transform& xf) const
{
	B3_NOT_USED(point);
	B3_NOT_USED(xf);
	return false;
}

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf, u32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	b3Triangle* triangle = m_mesh->triangles + index;

	b3Vec3 p1 = input.p1;
	b3Vec3 p2 = input.p2;
	b3Vec3 d = p2 - p1;

	b3Vec3 v1 = m_mesh->vertices[triangle->v1];
	b3Vec3 v2 = m_mesh->vertices[triangle->v2];
	b3Vec3 v3 = m_mesh->vertices[triangle->v3];

	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	n.Normalize();
	float32 offset = b3Dot(n, v1);

	float32 numerator = b3Dot(n, v1 - p1);
	float32 denominator = b3Dot(n, d);

	if (denominator == 0.0f)
	{
		return false;
	}

	float32 t = numerator / denominator;
	if (t < 0.0f || input.maxFraction < t)
	{
		return false;
	}

	b3Vec3 q = p1 + t * d;

	float32 w[4];
	b3BarycentricCoordinates(w, v1, v2, v3, q);

	if (w[0] > 0.0f && w[1] > 0.0f && w[2] > 0.0f)
	{
		output->fraction = t;
		if (numerator > 0.0f)
		{
			output->normal = -n;
		}
		else
		{
			output->normal = n;
		}
		return true;
	}

	return false;
}

struct b3MeshRayCastCallback
{
	float32 Report(const b3RayCastInput& input, u32 proxyId)
	{
		u32 index = mesh->m_mesh->tree.GetUserData(proxyId);
		b3RayCastOutput indexOutput;
		if (mesh->RayCast(&indexOutput, input, xf, index))
		{
			hit = true;
			if (indexOutput.fraction < output.fraction)
			{
				output = indexOutput;
			}
		}
		return 1.0f;
	}

	b3Transform xf;
	const b3MeshShape* mesh;
	bool hit;
	b3RayCastOutput output;
};

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	// Put the ray into the mesh's frame of reference.
	b3RayCastInput subInput;
	subInput.p1 = b3MulT(xf.rotation, input.p1 - xf.position);
	subInput.p2 = b3MulT(xf.rotation, input.p2 - xf.position);
	subInput.maxFraction = input.maxFraction;

	b3MeshRayCastCallback callback;
	callback.mesh = this;
	callback.hit = false;
	callback.output.fraction = B3_MAX_FLOAT;

	m_mesh->tree.RayCast(&callback, subInput);

	output->fraction = callback.output.fraction;
	output->normal = b3Mul(xf.rotation, callback.output.normal);

	return callback.hit;
}
