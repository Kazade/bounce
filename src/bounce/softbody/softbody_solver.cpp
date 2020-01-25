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
#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_force_solver.h>
#include <bounce/softbody/contacts/softbody_contact_solver.h>
#include <bounce/softbody/joints/softbody_anchor.h>

b3SoftBodySolver::b3SoftBodySolver(const b3SoftBodySolverDef& def)
{
	m_body = def.body;
	m_stack = &m_body->m_stackAllocator;
	m_mesh = m_body->m_mesh;
	m_nodes = m_body->m_nodes;
	m_elements = m_body->m_elements;
	
	m_shapeContactCapacity = m_body->m_contactManager.m_sphereAndShapeContactList.m_count;
	m_shapeContactCount = 0;
	m_shapeContacts = (b3SoftBodySphereAndShapeContact**)m_stack->Allocate(m_shapeContactCapacity * sizeof(b3SoftBodySphereAndShapeContact*));
	
	m_anchorCapacity = m_body->m_anchorList.m_count;
	m_anchorCount = 0;
	m_anchors = (b3SoftBodyAnchor**)m_stack->Allocate(m_anchorCapacity * sizeof(b3SoftBodyAnchor*));
}

b3SoftBodySolver::~b3SoftBodySolver()
{
	m_stack->Free(m_anchors);
	m_stack->Free(m_shapeContacts);
}

void b3SoftBodySolver::Add(b3SoftBodySphereAndShapeContact* c)
{
	m_shapeContacts[m_shapeContactCount++] = c;
}

void b3SoftBodySolver::Add(b3SoftBodyAnchor* a)
{
	m_anchors[m_anchorCount++] = a;
}

void b3SoftBodySolver::Solve(const b3SoftBodyTimeStep& step, const b3Vec3& gravity)
{
	{
		// Solve internal dynamics
		b3SoftBodyForceSolverDef forceSolverDef;
		forceSolverDef.step = step;
		forceSolverDef.body = m_body;

		b3SoftBodyForceSolver forceSolver(forceSolverDef);

		forceSolver.Solve(gravity);
	}

	// Copy node state to state buffer
	b3Vec3* positions = (b3Vec3*)m_stack->Allocate(m_mesh->vertexCount * sizeof(b3Vec3));
	b3Vec3* velocities = (b3Vec3*)m_stack->Allocate(m_mesh->vertexCount * sizeof(b3Vec3));

	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		positions[i] = m_nodes[i].m_position;
		velocities[i] = m_nodes[i].m_velocity;
	}

	b3SoftBodySolverData solverData;
	solverData.step = step;
	solverData.positions = positions;
	solverData.velocities = velocities;

	{
		// Solve constraints
		b3SoftBodyContactSolverDef contactSolverDef;
		contactSolverDef.step = step;
		contactSolverDef.allocator = m_stack;
		contactSolverDef.positions = positions;
		contactSolverDef.velocities = velocities;
		contactSolverDef.shapeContactCount = m_shapeContactCount;
		contactSolverDef.shapeContacts = m_shapeContacts;

		b3SoftBodyContactSolver contactSolver(contactSolverDef);

		{
			// Inititalize constraints
			contactSolver.InitializeShapeContactConstraints();

			for (u32 i = 0; i < m_anchorCount; ++i)
			{
				m_anchors[i]->InitializeConstraints(&solverData);
			}
		}

		{
			// Warm-start velocity constraint solver
			contactSolver.WarmStart();
			
			for (u32 i = 0; i < m_anchorCount; ++i)
			{
				m_anchors[i]->WarmStart(&solverData);
			}
		}

		{
			// Solve velocity constraints
			for (u32 i = 0; i < step.velocityIterations; ++i)
			{
				contactSolver.SolveShapeContactVelocityConstraints();
				
				for (u32 j = 0; j < m_anchorCount; ++j)
				{
					m_anchors[j]->SolveVelocityConstraints(&solverData);
				}
			}
		}

		{
			// Cache impulses for warm-starting
			contactSolver.StoreImpulses();
		}

		// Integrate velocities
		scalar h = step.dt;
		for (u32 i = 0; i < m_mesh->vertexCount; ++i)
		{
			positions[i] += h * velocities[i];
		}

		// Solve position constraints
		{
			bool positionSolved = false;
			for (u32 i = 0; i < step.positionIterations; ++i)
			{
				bool bodyContactsSolved = contactSolver.SolveShapeContactPositionConstraints();
				
				bool anchorsSolved = false;
				for (u32 j = 0; j < m_anchorCount; ++j)
				{
					bool anchorSolved = m_anchors[j]->SolvePositionConstraints(&solverData);
					anchorsSolved = anchorsSolved && anchorSolved;
				}

				positionSolved = bodyContactsSolved && anchorsSolved;

				if (positionSolved)
				{
					break;
				}
			}
		}
	}

	// Copy state buffers back to the nodes
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_position = positions[i];
		m_nodes[i].m_velocity = velocities[i];
	}

	m_stack->Free(velocities);
	m_stack->Free(positions);
}