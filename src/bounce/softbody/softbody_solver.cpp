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

#include <bounce/softbody/softbody_solver.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_force_solver.h>
#include <bounce/softbody/softbody_contact_solver.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/shapes/shape.h>

b3SoftBodySolver::b3SoftBodySolver(const b3SoftBodySolverDef& def)
{
	m_body = def.body;
	m_allocator = &m_body->m_stackAllocator;
	m_mesh = m_body->m_mesh;
	m_nodes = m_body->m_nodes;
	m_elements = m_body->m_elements;
	m_bodyContactCapacity = m_mesh->vertexCount;
	m_bodyContactCount = 0;
	m_bodyContacts = (b3NodeBodyContact**)m_allocator->Allocate(m_bodyContactCapacity * sizeof(b3NodeBodyContact*));
}

b3SoftBodySolver::~b3SoftBodySolver()
{
	m_allocator->Free(m_bodyContacts);
}

void b3SoftBodySolver::Add(b3NodeBodyContact* c)
{
	m_bodyContacts[m_bodyContactCount++] = c;
}

void b3SoftBodySolver::Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations)
{
	{
		// Solve internal dynamics
		b3SoftBodyForceSolverDef forceSolverDef;
		forceSolverDef.body = m_body;

		b3SoftBodyForceSolver forceSolver(forceSolverDef);

		forceSolver.Solve(dt, gravity);
	}

	// Copy node state to state buffer
	b3Vec3* positions = (b3Vec3*)m_allocator->Allocate(m_mesh->vertexCount * sizeof(b3Vec3));
	b3Vec3* velocities = (b3Vec3*)m_allocator->Allocate(m_mesh->vertexCount * sizeof(b3Vec3));

	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		positions[i] = m_nodes[i].m_position;
		velocities[i] = m_nodes[i].m_velocity;
	}

	{
		// Solve constraints
		b3SoftBodyContactSolverDef contactSolverDef;
		contactSolverDef.allocator = m_allocator;
		contactSolverDef.positions = positions;
		contactSolverDef.velocities = velocities;
		contactSolverDef.bodyContactCount = m_bodyContactCount;
		contactSolverDef.bodyContacts = m_bodyContacts;

		b3SoftBodyContactSolver contactSolver(contactSolverDef);

		{
			// Inititalize constraints
			contactSolver.InitializeBodyContactConstraints();
		}

		{
			// Warm-start velocity constraint solver
			contactSolver.WarmStart();
		}

		{
			// Solve velocity constraints
			for (u32 i = 0; i < velocityIterations; ++i)
			{
				contactSolver.SolveBodyContactVelocityConstraints();
			}
		}

		{
			// Cache impulses for warm-starting
			contactSolver.StoreImpulses();
		}

		// Integrate velocities
		float32 h = dt;
		for (u32 i = 0; i < m_mesh->vertexCount; ++i)
		{
			positions[i] += h * velocities[i];
		}

		// Solve position constraints
		{
			bool positionSolved = false;
			for (u32 i = 0; i < positionIterations; ++i)
			{
				bool bodyContactsSolved = contactSolver.SolveBodyContactPositionConstraints();

				if (bodyContactsSolved)
				{
					positionSolved = true;
					break;
				}
			}
		}

		// Synchronize bodies
		for (u32 i = 0; i < m_mesh->vertexCount; ++i)
		{
			b3SoftBodyNode* n = m_nodes + i;

			b3NodeBodyContact* c = &n->m_bodyContact;

			if (c->active == false)
			{
				continue;
			}

			b3Body* body = c->s2->GetBody();

			body->SynchronizeTransform();

			body->m_worldInvI = b3RotateToFrame(body->m_invI, body->m_xf.rotation);

			body->SynchronizeShapes();
		}
	}

	// Copy state buffers back to the nodes
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_position = positions[i];
		m_nodes[i].m_velocity = velocities[i];
	}

	m_allocator->Free(velocities);
	m_allocator->Free(positions);
}