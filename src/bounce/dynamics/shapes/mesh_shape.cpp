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
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/collision/shapes/triangle_hull.h>

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
	B3_ASSERT(index < m_mesh->triangleCount);
	const b3Triangle* triangle = m_mesh->triangles + index;
	b3Vec3 v1 = b3Mul(xf, m_mesh->vertices[triangle->v1]);
	b3Vec3 v2 = b3Mul(xf, m_mesh->vertices[triangle->v2]);
	b3Vec3 v3 = b3Mul(xf, m_mesh->vertices[triangle->v3]);

	output->m_lower = b3Min(b3Min(v1, v2), v3);
	output->m_upper = b3Max(b3Max(v1, v2), v3);
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
	B3_ASSERT(index < m_mesh->triangleCount);
	b3Triangle* triangle = m_mesh->triangles + index;
	b3Vec3 v1 = m_mesh->vertices[triangle->v1];
	b3Vec3 v2 = m_mesh->vertices[triangle->v2];
	b3Vec3 v3 = m_mesh->vertices[triangle->v3];

	b3TriangleHull hull(v1, v2, v3);

	b3HullShape hullShape;
	hullShape.m_body = m_body;
	hullShape.m_hull = &hull;
	hullShape.m_radius = B3_HULL_RADIUS;

	return hullShape.TestSphere(output, sphere, xf);
}

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf, u32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	b3Triangle* triangle = m_mesh->triangles + index;
	b3Vec3 v1 = m_mesh->vertices[triangle->v1];
	b3Vec3 v2 = m_mesh->vertices[triangle->v2];
	b3Vec3 v3 = m_mesh->vertices[triangle->v3];

	// Put the ray into the mesh's frame of reference.
	b3Vec3 p1 = b3MulT(xf, input.p1);
	b3Vec3 p2 = b3MulT(xf, input.p2);
	b3Vec3 d = p2 - p1;

	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	n.Normalize();
	
	float32 numerator = b3Dot(n, v1 - p1);
	float32 denominator = b3Dot(n, d);

	if (denominator == 0.0f)
	{
		return false;
	}

	float32 t = numerator / denominator;
		
	// Is the intersection not on the segment?
	if (t < 0.0f || input.maxFraction < t)
	{
		return false;
	}

	b3Vec3 q = p1 + t * d;

	// Barycentric coordinates for q
	b3Vec3 Q = q;
	b3Vec3 A = v1;
	b3Vec3 B = v2;
	b3Vec3 C = v3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;
	
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	float32 u = b3Dot(QB_x_QC, AB_x_AC);
	float32 v = b3Dot(QC_x_QA, AB_x_AC);
	float32 w = b3Dot(QA_x_QB, AB_x_AC);

	// This tolerance helps intersections lying on  
	// shared edges to not be missed.
	const float32 kTol = -0.005f;

	// Is the intersection on the triangle?
	if (u > kTol && v > kTol && w > kTol)
	{
		output->fraction = t;
		
		// Does the ray start from below or above the triangle?
		if (numerator > 0.0f)
		{
			output->normal = -b3Mul(xf.rotation, n);
		}
		else
		{
			output->normal = b3Mul(xf.rotation, n);
		}
		
		return true;
	}

	return false;
}

struct b3MeshRayCastCallback
{
	float32 Report(const b3RayCastInput& subInput, u32 proxyId)
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
		
		return 1.0f;
	}

	b3RayCastInput input;
	const b3MeshShape* mesh;
	b3Transform xf;
	
	bool hit;
	b3RayCastOutput output;
};

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	b3MeshRayCastCallback callback;
	callback.input = input;
	callback.mesh = this;
	callback.xf = xf;
	callback.hit = false;
	callback.output.fraction = B3_MAX_FLOAT;
	
	b3RayCastInput subInput;
	subInput.p1 = b3MulT(xf, input.p1);
	subInput.p2 = b3MulT(xf, input.p2);
	subInput.maxFraction = input.maxFraction;
	m_mesh->tree.RayCast(&callback, subInput);

	output->fraction = callback.output.fraction;
	output->normal = callback.output.normal;

	return callback.hit;
}