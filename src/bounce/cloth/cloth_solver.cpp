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

#include <bounce/cloth/cloth_solver.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/cloth/contacts/cloth_contact_solver.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_time_step.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/forces/element_force.h>
#include <bounce/common/memory/stack_allocator.h>

b3ClothSolver::b3ClothSolver(const b3ClothSolverDef& def)
{
	m_stack = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3ClothParticle**)m_stack->Allocate(m_particleCapacity * sizeof(b3ClothParticle*));

	m_forceCapacity = def.forceCapacity;
	m_forceCount = 0;
	m_forces = (b3Force**)m_stack->Allocate(m_forceCapacity * sizeof(b3Force*));;

	m_shapeContactCapacity = def.shapeContactCapacity;
	m_shapeContactCount = 0;
	m_shapeContacts = (b3ClothSphereAndShapeContact**)m_stack->Allocate(m_shapeContactCapacity * sizeof(b3ClothSphereAndShapeContact*));

	m_triangleContactCapacity = def.triangleContactCapacity;
	m_triangleContactCount = 0;
	m_triangleContacts = (b3ClothSphereAndTriangleContact**)m_stack->Allocate(m_triangleContactCapacity * sizeof(b3ClothSphereAndTriangleContact*));
	
	m_capsuleContactCapacity = def.capsuleContactCapacity;
	m_capsuleContactCount = 0;
	m_capsuleContacts = (b3ClothCapsuleAndCapsuleContact* *)m_stack->Allocate(m_capsuleContactCapacity * sizeof(b3ClothCapsuleAndCapsuleContact*));
}

b3ClothSolver::~b3ClothSolver()
{
	m_stack->Free(m_capsuleContacts);
	m_stack->Free(m_triangleContacts);
	m_stack->Free(m_shapeContacts);
	m_stack->Free(m_forces);
	m_stack->Free(m_particles);
}

void b3ClothSolver::Add(b3ClothParticle* p)
{
	p->m_solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3ClothSolver::Add(b3Force* f)
{
	m_forces[m_forceCount++] = f;
}

void b3ClothSolver::Add(b3ClothSphereAndShapeContact* c)
{
	m_shapeContacts[m_shapeContactCount++] = c;
}

void b3ClothSolver::Add(b3ClothSphereAndTriangleContact* c)
{
	m_triangleContacts[m_triangleContactCount++] = c;
}

void b3ClothSolver::Add(b3ClothCapsuleAndCapsuleContact* c)
{
	m_capsuleContacts[m_capsuleContactCount++] = c;
}

void b3ClothSolver::Solve(const b3ClothTimeStep& step, const b3Vec3& gravity)
{
	{
		// Solve internal dynamics
		b3ClothForceSolverDef forceSolverDef;
		forceSolverDef.step = step;
		forceSolverDef.stack = m_stack;
		forceSolverDef.particleCount = m_particleCount;
		forceSolverDef.particles = m_particles;
		forceSolverDef.forceCount = m_forceCount;
		forceSolverDef.forces = m_forces;

		b3ClothForceSolver forceSolver(forceSolverDef);

		forceSolver.Solve(gravity);
	}
	
	// Copy particle state to state buffer
	b3Vec3* positions = (b3Vec3*)m_stack->Allocate(m_particleCount * sizeof(b3Vec3));
	b3Vec3* velocities = (b3Vec3*)m_stack->Allocate(m_particleCount * sizeof(b3Vec3));
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		positions[i] = m_particles[i]->m_position;
		velocities[i] = m_particles[i]->m_velocity;
	}

	{
		// Solve constraints
		b3ClothContactSolverDef contactSolverDef;
		contactSolverDef.step = step;
		contactSolverDef.allocator = m_stack;
		contactSolverDef.positions = positions;
		contactSolverDef.velocities = velocities;
		contactSolverDef.shapeContactCount = m_shapeContactCount;
		contactSolverDef.shapeContacts = m_shapeContacts;
		contactSolverDef.triangleContactCount = m_triangleContactCount;
		contactSolverDef.triangleContacts = m_triangleContacts;
		contactSolverDef.capsuleContactCount = m_capsuleContactCount;
		contactSolverDef.capsuleContacts = m_capsuleContacts;

		b3ClothContactSolver contactSolver(contactSolverDef);

		{
			// Initialize constraints
			contactSolver.InitializeShapeContactConstraints();
			contactSolver.InitializeTriangleContactConstraints();
			contactSolver.InitializeCapsuleContactConstraints();
		}

		{
			// Warm start velocity constraints
			contactSolver.WarmStartShapeContactConstraints();
			contactSolver.WarmStartTriangleContactConstraints();
			contactSolver.WarmStartCapsuleContactConstraints();
		}

		{
			// Solve velocity constraints
			for (u32 i = 0; i < step.velocityIterations; ++i)
			{
				contactSolver.SolveShapeContactVelocityConstraints();
				contactSolver.SolveTriangleContactVelocityConstraints();
				contactSolver.SolveCapsuleContactVelocityConstraints();
			}
		}

		{
			// Cache impulses for warm-starting
			contactSolver.StoreImpulses();
		}

		// Integrate positions
		scalar h = step.dt;
		for (u32 i = 0; i < m_particleCount; ++i)
		{
			positions[i] += h * velocities[i];
		}

		{
			// Solve position constraints
			bool positionSolved = false;
			for (u32 i = 0; i < step.positionIterations; ++i)
			{
				bool bodyContactsSolved = contactSolver.SolveShapeContactPositionConstraints();
				bool triangleContactsSolved = contactSolver.SolveTriangleContactPositionConstraints();
				bool capsuleContactsSolved = contactSolver.SolveCapsuleContactPositionConstraints();

				if (bodyContactsSolved && triangleContactsSolved && capsuleContactsSolved)
				{
					// Early out if the position errors are small.
					positionSolved = true;
					break;
				}
			}
		}
	}

	// Copy state buffers back to the particles
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->m_position = positions[i];
		m_particles[i]->m_velocity = velocities[i];
	}

	m_stack->Free(velocities);
	m_stack->Free(positions);
}