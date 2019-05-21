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

#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>

#include <bounce/softbody/softbody_solver.h>

#include <bounce/collision/collision.h>

#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/shapes/shape.h>

#include <bounce/common/draw.h>

static B3_FORCE_INLINE void b3Mul(float32* C, float32* A, u32 AM, u32 AN, float32* B, u32 BM, u32 BN)
{
	B3_ASSERT(AN == BM);

	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < BN; ++j)
		{
			C[i + AM * j] = 0.0f;

			for (u32 k = 0; k < AN; ++k)
			{
				C[i + AM * j] += A[i + AM * k] * B[k + BM * j];
			}
		}
	}
}

static B3_FORCE_INLINE void b3Transpose(float32* B, float32* A, u32 AM, u32 AN)
{
	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < AN; ++j)
		{
			B[j + AN * i] = A[i + AM * j];
		}
	}
}

// Compute the elasticity matrix given Young modulus and Poisson's ratio
// This is a 6 x 6 matrix 
static B3_FORCE_INLINE void b3ComputeD(float32 out[36], float32 E, float32 nu)
{
	float32 lambda = (nu * E) / ((1 + nu) * (1 - 2 * nu));
	float32 mu = E / (2 * (1 + nu));

	float32 D[36] =
	{
		lambda + 2 * mu, lambda,          lambda,          0,  0,  0,
		lambda,          lambda + 2 * mu, lambda,          0,  0,  0,
		lambda,          lambda,          lambda + 2 * mu, 0,  0,  0,
		0, 				 0, 			  0, 			   mu, 0,  0,
		0, 				 0, 			  0, 			   0,  mu, 0,
		0, 			 	 0, 			  0, 			   0,  0,  mu
	};

	for (u32 i = 0; i < 36; ++i)
	{
		out[i] = D[i];
	}
}

// Compute derivatives of shape functions N
static B3_FORCE_INLINE void b3ComputeBVec(b3Vec3 out[4], 
	const b3Vec3& e10, 
	const b3Vec3& e20, 
	const b3Vec3& e30,
	float32 V)
{
	float32 inv6V = 1.0f / (6.0f * V);

	b3Vec3* B = out;

	B[1][0] = (e20[2] * e30[1] - e20[1] * e30[2]) * inv6V;
	B[2][0] = (e10[1] * e30[2] - e10[2] * e30[1]) * inv6V;
	B[3][0] = (e10[2] * e20[1] - e10[1] * e20[2]) * inv6V;
	B[0][0] = -B[1][0] - B[2][0] - B[3][0];

	B[1][1] = (e20[0] * e30[2] - e20[2] * e30[0]) * inv6V;
	B[2][1] = (e10[2] * e30[0] - e10[0] * e30[2]) * inv6V;
	B[3][1] = (e10[0] * e20[2] - e10[2] * e20[0]) * inv6V;
	B[0][1] = -B[1][1] - B[2][1] - B[3][1]; 

	B[1][2] = (e20[1] * e30[0] - e20[0] * e30[1]) * inv6V;
	B[2][2] = (e10[0] * e30[1] - e10[1] * e30[0]) * inv6V;
	B[3][2] = (e10[1] * e20[0] - e10[0] * e20[1]) * inv6V;
	B[0][2] = -B[1][2] - B[2][2] - B[3][2]; 
}

// Convert a B vector to its corresponding matrix form.
// This is a 6 x 3 matrix.
static B3_FORCE_INLINE void b3ComputeBMat(float32 out[18], const b3Vec3& v)
{
	float32 bi = v.x;
	float32 ci = v.y;
	float32 di = v.z;

	float32 B[18] =
	{
		bi, 0, 0, ci, di, 0,
		0, ci, 0, bi, 0, di,
		0, 0, di, 0, bi, ci
	};

	for (u32 i = 0; i < 18; ++i)
	{
		out[i] = B[i];
	}
}

b3SoftBody::b3SoftBody(const b3SoftBodyDef& def)
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_mesh = def.mesh;
	m_density = def.density;
	m_E = def.E;
	m_nu = def.nu;
	m_gravity.SetZero();
	m_world = nullptr;

	const b3SoftBodyMesh* m = m_mesh;

	m_nodes = (b3SoftBodyNode*)b3Alloc(m->vertexCount * sizeof(b3SoftBodyNode));

	// Initialize nodes
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		n->m_body = this;
		n->m_type = e_dynamicSoftBodyNode;
		n->m_position = m->vertices[i];
		n->m_velocity.SetZero();
		n->m_force.SetZero();
		n->m_mass = 0.0f;
		n->m_invMass = 0.0f;
		n->m_radius = 0.0f;
		n->m_friction = 0.0f;
		n->m_userData = nullptr;
		n->m_vertex = i;
		n->m_bodyContact.active = false;
	}

	// Compute mass
	ComputeMass();

	// Initialize elements
	m_elements = (b3SoftBodyElement*)b3Alloc(m->tetrahedronCount * sizeof(b3SoftBodyElement));
	for (u32 ei = 0; ei < m->tetrahedronCount; ++ei)
	{
		b3SoftBodyMeshTetrahedron* mt = m->tetrahedrons + ei;
		b3SoftBodyElement* e = m_elements + ei;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		b3Vec3 p1 = m->vertices[v1];
		b3Vec3 p2 = m->vertices[v2];
		b3Vec3 p3 = m->vertices[v3];
		b3Vec3 p4 = m->vertices[v4];

		float32 V = b3Volume(p1, p2, p3, p4);

		B3_ASSERT(V > 0.0f);

		b3Vec3 e10 = p2 - p1;
		b3Vec3 e20 = p3 - p1;
		b3Vec3 e30 = p4 - p1;

		b3Mat33 E(e10, e20, e30);

		e->invE = b3Inverse(E);

		// 6 x 6
		float32 D[36];
		b3ComputeD(D, m_E, m_nu);

		b3Vec3 B[4];
		b3ComputeBVec(B, e10, e20, e30, V);

		for (u32 i = 0; i < 4; ++i)
		{
			// 6 x 3
			float32 B_i[18];
			b3ComputeBMat(B_i, B[i]);

			// 3 x 6
			float32 B_i_T[18];
			b3Transpose(B_i_T, B_i, 6, 3);

			for (u32 j = 0; j < 4; ++j)
			{
				// 6 x 3
				float32 B_j[18];
				b3ComputeBMat(B_j, B[j]);

				// 6 x 3
				float32 D_B_j[18];
				b3Mul(D_B_j, D, 6, 6, B_j, 6, 3);

				// 3 x 3
				b3Mat33& Ke = e->K[i + 4 * j];
				b3Mul(&Ke.x.x, B_i_T, 3, 6, D_B_j, 6, 3);

				Ke = V * Ke;
			}
		}
	}

	// Initialize triangles
	m_triangles = (b3SoftBodyTriangle*)b3Alloc(4 * m_mesh->tetrahedronCount * sizeof(b3SoftBodyTriangle));
	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* mt = m_mesh->tetrahedrons + i;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		b3SoftBodyTriangle* t1 = m_triangles + 4 * i + 0;
		b3SoftBodyTriangle* t2 = m_triangles + 4 * i + 1;
		b3SoftBodyTriangle* t3 = m_triangles + 4 * i + 2;
		b3SoftBodyTriangle* t4 = m_triangles + 4 * i + 3;

		t1->v1 = v1;
		t1->v2 = v2;
		t1->v3 = v3;
		t1->tetrahedron = i;

		t2->v1 = v1;
		t2->v2 = v3;
		t2->v3 = v4;
		t2->tetrahedron = i;

		t3->v1 = v1;
		t3->v2 = v4;
		t3->v3 = v2;
		t3->tetrahedron = i;

		t4->v1 = v2;
		t4->v2 = v4;
		t4->v3 = v3;
		t4->tetrahedron = i;
	}
}

b3SoftBody::~b3SoftBody()
{
	b3Free(m_nodes);
	b3Free(m_elements);
	b3Free(m_triangles);
}

bool b3SoftBody::RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	u32 triangle = ~0;
	u32 tetrahedron = ~0;

	b3RayCastOutput output0;
	output0.fraction = B3_MAX_FLOAT;

	for (u32 i = 0; i < 4 * m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyTriangle* t = m_triangles + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;

		b3RayCastOutput subOutput;
		if (b3RayCast(&subOutput, &input, v1, v2, v3))
		{
			if (subOutput.fraction < output0.fraction)
			{
				triangle = i;
				tetrahedron = t->tetrahedron;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}
	}

	if (tetrahedron != ~0)
	{
		output->tetrahedron = tetrahedron;
		output->v1 = m_triangles[triangle].v1;
		output->v2 = m_triangles[triangle].v2;
		output->v3 = m_triangles[triangle].v3;
		output->fraction = output0.fraction;
		output->normal = output0.normal;

		return true;
	}

	return false;
}

b3SoftBodyNode* b3SoftBody::GetVertexNode(u32 i)
{
	B3_ASSERT(i < m_mesh->vertexCount);
	return m_nodes + i;
}

float32 b3SoftBody::GetEnergy() const
{
	float32 E = 0.0f;
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		E += n->m_mass * b3Dot(n->m_velocity, n->m_velocity);
	}
	return 0.5f * E;
}

void b3SoftBody::ComputeMass()
{
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		n->m_mass = 0.0f;
		n->m_invMass = 0.0f;
	}

	const float32 inv4 = 1.0f / 4.0f;
	const float32 rho = m_density;

	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* tetrahedron = m_mesh->tetrahedrons + i;

		b3Vec3 v1 = m_mesh->vertices[tetrahedron->v1];
		b3Vec3 v2 = m_mesh->vertices[tetrahedron->v2];
		b3Vec3 v3 = m_mesh->vertices[tetrahedron->v3];
		b3Vec3 v4 = m_mesh->vertices[tetrahedron->v4];

		float32 volume = b3Volume(v1, v2, v3, v4);
		B3_ASSERT(volume > 0.0f);

		float32 mass = rho * volume;

		b3SoftBodyNode* n1 = m_nodes + tetrahedron->v1;
		b3SoftBodyNode* n2 = m_nodes + tetrahedron->v2;
		b3SoftBodyNode* n3 = m_nodes + tetrahedron->v3;
		b3SoftBodyNode* n4 = m_nodes + tetrahedron->v4;

		n1->m_mass += inv4 * mass;
		n2->m_mass += inv4 * mass;
		n3->m_mass += inv4 * mass;
		n4->m_mass += inv4 * mass;
	}

	// Invert
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		B3_ASSERT(n->m_mass > 0.0f);
		n->m_invMass = 1.0f / n->m_mass;
	}
}

void b3SoftBody::UpdateContacts()
{
	B3_PROFILE("Soft Body Update Contacts");

	// Is there a world attached to this soft body?
	if (m_world == nullptr)
	{
		return;
	}

	// Create contacts 
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		b3Sphere s1;
		s1.vertex = n->m_position;
		s1.radius = n->m_radius;

		// Find the deepest penetration
		b3Shape* bestShape = nullptr;
		float32 bestSeparation = 0.0f;
		b3Vec3 bestPoint(0.0f, 0.0f, 0.0f);
		b3Vec3 bestNormal(0.0f, 0.0f, 0.0f);

		for (b3Body* body = m_world->GetBodyList().m_head; body; body = body->GetNext())
		{
			if (n->m_type != e_dynamicSoftBodyNode)
			{
				continue;
			}

			if (body->GetType() != e_staticBody)
			{
				//continue;
			}

			b3Transform xf = body->GetTransform();
			for (b3Shape* shape = body->GetShapeList().m_head; shape; shape = shape->GetNext())
			{
				b3TestSphereOutput output;
				if (shape->TestSphere(&output, s1, xf))
				{
					if (output.separation < bestSeparation)
					{
						bestShape = shape;
						bestSeparation = output.separation;
						bestPoint = output.point;
						bestNormal = output.normal;
					}
				}
			}
		}

		if (bestShape == nullptr)
		{
			n->m_bodyContact.active = false;
			continue;
		}

		b3Shape* shape = bestShape;
		b3Body* body = shape->GetBody();
		float32 separation = bestSeparation;
		b3Vec3 point = bestPoint;
		b3Vec3 normal = -bestNormal;

		b3NodeBodyContact* c = &n->m_bodyContact;

		b3NodeBodyContact c0 = *c;

		c->active = true;
		c->n1 = n;
		c->s2 = shape;
		c->normal1 = normal;
		c->localPoint1.SetZero();
		c->localPoint2 = body->GetLocalPoint(point);
		c->t1 = b3Perp(normal);
		c->t2 = b3Cross(c->t1, normal);
		c->normalImpulse = 0.0f;
		c->tangentImpulse.SetZero();

		if (c0.active == true)
		{
			c->normalImpulse = c0.normalImpulse;
			c->tangentImpulse = c0.tangentImpulse;
		}
	}
}

void b3SoftBody::Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Soft Body Solve");

	b3SoftBodySolverDef def;
	def.stack = &m_stackAllocator;
	def.mesh = m_mesh;
	def.nodes = m_nodes;
	def.elements = m_elements;

	b3SoftBodySolver solver(def);

	solver.Solve(dt, gravity, velocityIterations, positionIterations);
}

void b3SoftBody::Step(float32 dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Soft Body Step");

	// Update contacts
	UpdateContacts();

	// Integrate state, solve constraints. 
	if (dt > 0.0f)
	{
		Solve(dt, m_gravity, velocityIterations, positionIterations);
	}

	// Clear forces
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_force.SetZero();
	}
}

void b3SoftBody::Draw() const
{
	const b3SoftBodyMesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		b3Vec3 v = n->m_position;

		if (n->m_type == e_staticSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, 4.0f, b3Color_white);
		}

		if (n->m_type == e_dynamicSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, 4.0f, b3Color_green);
		}
	}

	for (u32 i = 0; i < m->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* t = m->tetrahedrons + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;
		b3Vec3 v4 = m_nodes[t->v4].m_position;

		b3Vec3 c = (v1 + v2 + v3 + v4) / 4.0f;

		float32 s = 0.9f;

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;
		v4 = s * (v4 - c) + c;

		// v1, v2, v3
		b3Draw_draw->DrawTriangle(v1, v2, v3, b3Color_black);
		
		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n1, v1, v2, v3, b3Color_blue);

		// v1, v3, v4
		b3Draw_draw->DrawTriangle(v1, v3, v4, b3Color_black);
		
		b3Vec3 n2 = b3Cross(v3 - v1, v4 - v1);
		n2.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n2, v1, v3, v4, b3Color_blue);

		// v1, v4, v2
		b3Draw_draw->DrawTriangle(v1, v4, v2, b3Color_black);
		
		b3Vec3 n3 = b3Cross(v4 - v1, v2 - v1);
		n3.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n3, v1, v4, v2, b3Color_blue);

		// v2, v4, v3
		b3Draw_draw->DrawTriangle(v2, v4, v3, b3Color_black);
		
		b3Vec3 n4 = b3Cross(v4 - v2, v3 - v2);
		n4.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n4, v2, v4, v3, b3Color_blue);
	}
}