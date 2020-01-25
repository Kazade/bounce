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
#include <bounce/softbody/softbody_element.h>
#include <bounce/softbody/softbody_solver.h>
#include <bounce/softbody/softbody_time_step.h>
#include <bounce/softbody/shapes/softbody_sphere_shape.h>
#include <bounce/softbody/shapes/softbody_world_shape.h>
#include <bounce/softbody/joints/softbody_anchor.h>
#include <bounce/collision/collision.h>
#include <bounce/sparse/sparse_mat33_pattern.h>
#include <bounce/common/draw.h>

b3SoftBody::b3SoftBody(const b3SoftBodyDef& def) : 
	m_sphereShapeBlocks(sizeof(b3SoftBodySphereShape)),
	m_worldShapeBlocks(sizeof(b3SoftBodyWorldShape))
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(scalar(0) < def.density);
	B3_ASSERT(scalar(0) <= def.massDamping);
	B3_ASSERT(scalar(0) <= def.stiffnessDamping);

	m_mesh = def.mesh;
	m_density = def.density;
	m_massDamping = def.massDamping;
	m_stiffnessDamping = def.stiffnessDamping;
	
	m_gravity.SetZero();
	m_contactManager.m_body = this;
	m_inv_dt0 = scalar(0);

	const b3SoftBodyMesh* m = m_mesh;

	// Initialize nodes
	m_nodes = (b3SoftBodyNode*)b3Alloc(m->vertexCount * sizeof(b3SoftBodyNode));
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3SoftBodyNode* n = new (m_nodes + i) b3SoftBodyNode();

		n->m_body = this;
		n->m_type = e_dynamicSoftBodyNode;
		n->m_position = m->vertices[i];
		n->m_velocity.SetZero();
		n->m_force.SetZero();
		n->m_translation.SetZero();
		n->m_mass = scalar(0);
		n->m_invMass = scalar(0);
		n->m_meshIndex = i;

		b3SoftBodySphereShapeDef sd;
		sd.node = n;
		sd.radius = def.radius;
		sd.density = def.density;
		sd.friction = def.friction;
		
		CreateSphereShape(sd);
	}

	// Compute mass
	ComputeMass();

	// Initialize elements
	m_KP = new (b3Alloc(sizeof(b3SparseMat33Pattern))) b3SparseMat33Pattern(m->vertexCount);
	m_elements = (b3SoftBodyElement*)b3Alloc(m->tetrahedronCount * sizeof(b3SoftBodyElement));
	for (u32 ei = 0; ei < m->tetrahedronCount; ++ei)
	{
		b3SoftBodyMeshTetrahedron* mt = m->tetrahedrons + ei;
		b3SoftBodyElement* e = new (m_elements + ei) b3SoftBodyElement();

		e->m_body = this;
		e->m_q.SetIdentity();
		e->m_E = def.E;
		e->m_nu = def.nu;
		e->m_c_yield = def.c_yield;
		e->m_c_creep = def.c_creep;
		e->m_c_max = def.c_max;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		u32 vs[4] = { v1, v2, v3, v4 };
		
		for (u32 i = 0; i < 4; ++i)
		{
			u32 vi = vs[i];

			for (u32 j = 0; j < 4; ++j)
			{
				u32 vj = vs[j];
				
				e->m_Kp[i + 4 * j] = m_KP->CreateElement(vi, vj);
			}
		}
		
		b3Vec3 p1 = m->vertices[v1];
		b3Vec3 p2 = m->vertices[v2];
		b3Vec3 p3 = m->vertices[v3];
		b3Vec3 p4 = m->vertices[v4];

		e->m_V = b3Volume(p1, p2, p3, p4);

		B3_ASSERT(e->m_V > scalar(0));

		b3Vec3 e1 = p2 - p1;
		b3Vec3 e2 = p3 - p1;
		b3Vec3 e3 = p4 - p1;

		b3Mat33 E(e1, e2, e3);

		e->m_invE = b3Inverse(E);

		for (u32 i = 0; i < 6; ++i)
		{
			e->m_epsilon_plastic[i] = scalar(0);
		}

		e->ComputeMatrices();
	}
}

b3SoftBody::~b3SoftBody()
{
	// Destroy nodes
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].~b3SoftBodyNode();
	}
	b3Free(m_nodes);
	
	// Destroy elements
	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		m_elements[i].~b3SoftBodyElement();
	}
	b3Free(m_elements);
	
	// Destroy patterns
	m_KP->~b3SparseMat33Pattern();
	b3Free(m_KP);

	// Destroy joints
	b3SoftBodyAnchor* a = m_anchorList.m_head;
	while (a)
	{
		b3SoftBodyAnchor* boom = a;
		a = a->m_next;

		boom->~b3SoftBodyAnchor();
		b3Free(boom);
	}
}

b3SoftBodySphereShape* b3SoftBody::CreateSphereShape(const b3SoftBodySphereShapeDef& def)
{
	// Does the shape exist already?
	for (b3SoftBodySphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_node == def.node)
		{
			return s;
		}
	}

	void* mem = m_sphereShapeBlocks.Allocate();
	b3SoftBodySphereShape* s = new (mem)b3SoftBodySphereShape();

	s->m_type = e_softBodySphereShape;
	s->m_radius = def.radius;
	s->m_density = def.density;
	s->m_friction = def.friction;
	s->m_node = def.node;
	s->m_body = this;

	// Create broadphase
	b3AABB aabb = s->ComputeAABB();
	s->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, s);

	// Put into list
	m_sphereShapeList.PushFront(s);

	return s;
}

void b3SoftBody::DestroySphereShape(b3SoftBodySphereShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remove from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	// Remove from list
	m_sphereShapeList.Remove(shape);
	
	// Free memory
	shape->~b3SoftBodySphereShape();
	m_sphereShapeBlocks.Free(shape);
}

b3SoftBodyWorldShape* b3SoftBody::CreateWorldShape(const b3SoftBodyWorldShapeDef& def)
{
	// Does the shape exist already?
	for (b3SoftBodyWorldShape* s = m_worldShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_shape == def.shape)
		{
			return s;
		}
	}

	void* mem = m_worldShapeBlocks.Allocate();
	b3SoftBodyWorldShape* s = new (mem)b3SoftBodyWorldShape();

	s->m_type = e_softBodyWorldShape;
	s->m_shape = def.shape;
	s->m_body = this;

	// Create broadphase
	b3AABB aabb = s->ComputeAABB();
	s->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, s);

	// Push to list
	m_worldShapeList.PushFront(s);

	return s;
}

void b3SoftBody::DestroyWorldShape(b3SoftBodyWorldShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remove from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	// Remove from list
	m_worldShapeList.Remove(shape);

	// Free memory
	shape->~b3SoftBodyWorldShape();
	m_worldShapeBlocks.Free(shape);
}

b3SoftBodyAnchor* b3SoftBody::CreateAnchor(const b3SoftBodyAnchorDef& def)
{
	void* p = (b3SoftBodyAnchor*)b3Alloc(sizeof(b3SoftBodyAnchor));
	b3SoftBodyAnchor* a = new (p) b3SoftBodyAnchor(def);
	m_anchorList.PushFront(a);
	return a;
}

void b3SoftBody::DestroyAnchor(b3SoftBodyAnchor* anchor)
{
	m_anchorList.Remove(anchor);
	anchor->~b3SoftBodyAnchor();
	b3Free(anchor);
}

bool b3SoftBody::RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	u32 triangle0 = u32(~0);
	b3RayCastOutput output0;
	output0.fraction = B3_MAX_SCALAR;

	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3SoftBodyMeshTriangle* t = m_mesh->triangles + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;

		b3RayCastOutput subOutput;
		if (b3RayCast(&subOutput, &input, v1, v2, v3))
		{
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = i;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}
	}

	if (triangle0 != ~0)
	{
		output->triangle = triangle0;
		output->fraction = output0.fraction;
		output->normal = output0.normal;

		return true;
	}

	return false;
}

b3SoftBodyNode* b3SoftBody::GetNode(u32 i)
{
	B3_ASSERT(i < m_mesh->vertexCount);
	return m_nodes + i;
}

b3SoftBodyElement* b3SoftBody::GetElement(u32 i)
{
	B3_ASSERT(i < m_mesh->tetrahedronCount);
	return m_elements + i;
}

scalar b3SoftBody::GetEnergy() const
{
	scalar E = scalar(0);
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		E += n->m_mass * b3Dot(n->m_velocity, n->m_velocity);
	}
	return scalar(0.5) * E;
}

void b3SoftBody::ComputeMass()
{
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		n->m_mass = scalar(0);
		n->m_invMass = scalar(0);
	}

	const scalar inv4 = scalar(1) / scalar(4);
	const scalar rho = m_density;

	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* tetrahedron = m_mesh->tetrahedrons + i;

		b3Vec3 v1 = m_mesh->vertices[tetrahedron->v1];
		b3Vec3 v2 = m_mesh->vertices[tetrahedron->v2];
		b3Vec3 v3 = m_mesh->vertices[tetrahedron->v3];
		b3Vec3 v4 = m_mesh->vertices[tetrahedron->v4];

		scalar volume = b3Volume(v1, v2, v3, v4);
		B3_ASSERT(volume > scalar(0));

		scalar mass = rho * volume;

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
		B3_ASSERT(n->m_mass > scalar(0));
		n->m_invMass = scalar(1) / n->m_mass;
	}
}

void b3SoftBody::Solve(const b3SoftBodyTimeStep& step)
{
	B3_PROFILE("Soft Body Solve");

	b3SoftBodySolverDef def;
	def.body = this;

	b3SoftBodySolver solver(def);

	// Push the shape contacts
	for (b3SoftBodySphereAndShapeContact* c = m_contactManager.m_sphereAndShapeContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	// Push the anchors
	for (b3SoftBodyAnchor* a = m_anchorList.m_head; a; a = a->m_next)
	{
		solver.Add(a);
	}

	solver.Solve(step, m_gravity);
}

void b3SoftBody::Step(scalar dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Soft Body Step");

	// Update contacts
	m_contactManager.UpdateContacts();

	// Time step parameters
	b3SoftBodyTimeStep step;
	step.dt = dt;
	step.velocityIterations = velocityIterations;
	step.positionIterations = positionIterations;
	step.inv_dt = dt > scalar(0) ? scalar(1) / dt : scalar(0);
	step.dt_ratio = m_inv_dt0 * dt;

	// Integrate state, solve constraints. 
	if (step.dt > scalar(0))
	{
		Solve(step);
	}

	if (step.dt > scalar(0))
	{
		m_inv_dt0 = step.inv_dt;
	}

	// Clear state buffers
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_force.SetZero();
		m_nodes[i].m_translation.SetZero();
	}
	
	// Synchronize shapes
	for (b3SoftBodyWorldShape* s = m_worldShapeList.m_head; s; s = s->m_next)
	{
		s->Synchronize(b3Vec3_zero);
	}

	// Synchronize spheres
	for (b3SoftBodySphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		s->Synchronize(b3Vec3_zero);
	}
	
	// Find new contacts
	m_contactManager.FindNewContacts();
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
			b3Draw_draw->DrawPoint(v, scalar(4), b3Color_white);
		}

		if (n->m_type == e_kinematicSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, scalar(4), b3Color_blue);
		}

		if (n->m_type == e_dynamicSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, scalar(4), b3Color_green);
		}
	}

	for (u32 i = 0; i < m->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* t = m->tetrahedrons + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;
		b3Vec3 v4 = m_nodes[t->v4].m_position;

		b3Vec3 c = (v1 + v2 + v3 + v4) / scalar(4);

		scalar s(0.9);

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;
		v4 = s * (v4 - c) + c;

		// v1, v2, v3
		b3Draw_draw->DrawTriangle(v1, v2, v3, b3Color_black);

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		// v1, v3, v4
		b3Draw_draw->DrawTriangle(v1, v3, v4, b3Color_black);

		b3Vec3 n2 = b3Cross(v3 - v1, v4 - v1);
		n2.Normalize();
		b3Draw_draw->DrawSolidTriangle(n2, v1, v3, v4, b3Color_blue);

		// v1, v4, v2
		b3Draw_draw->DrawTriangle(v1, v4, v2, b3Color_black);

		b3Vec3 n3 = b3Cross(v4 - v1, v2 - v1);
		n3.Normalize();
		b3Draw_draw->DrawSolidTriangle(n3, v1, v4, v2, b3Color_blue);

		// v2, v4, v3
		b3Draw_draw->DrawTriangle(v2, v4, v3, b3Color_black);

		b3Vec3 n4 = b3Cross(v4 - v2, v3 - v2);
		n4.Normalize();
		b3Draw_draw->DrawSolidTriangle(n4, v2, v4, v3, b3Color_blue);
	}

	/*
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3SoftBodyMeshTriangle* t = m->triangles + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;

		b3Vec3 c = (v1 + v2 + v3) / scalar(3);

		scalar s = scalar(0.9);

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;

		b3Draw_draw->DrawTriangle(v1, v2, v3, b3Color_black);

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();
		b3Draw_draw->DrawSolidTriangle(n, v1, v2, v3, b3Color_blue);
		
		b3Draw_draw->DrawSegment(c, c + n, b3Color_white);
	}
	*/

	for (b3SoftBodyAnchor* a = m_anchorList.m_head; a; a = a->m_next)
	{
		b3Vec3 pA = a->GetAnchorA();
		b3Vec3 pB = a->GetAnchorB();

		b3Draw_draw->DrawPoint(pA, scalar(4), b3Color_red);

		b3Draw_draw->DrawPoint(pB, scalar(4), b3Color_green);

		b3Draw_draw->DrawSegment(pA, pB, b3Color_yellow);
	}
}