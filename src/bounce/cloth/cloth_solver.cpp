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
#include <bounce/cloth/cloth_contact_solver.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/particle.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>

b3ClothSolver::b3ClothSolver(const b3ClothSolverDef& def)
{
	m_allocator = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3Particle**)m_allocator->Allocate(m_particleCapacity * sizeof(b3Particle*));

	m_forceCapacity = def.forceCapacity;
	m_forceCount = 0;
	m_forces = (b3Force**)m_allocator->Allocate(m_forceCapacity * sizeof(b3Force*));;

	m_bodyContactCapacity = def.bodyContactCapacity;
	m_bodyContactCount = 0;
	m_bodyContacts = (b3ParticleBodyContact**)m_allocator->Allocate(m_bodyContactCapacity * sizeof(b3ParticleBodyContact*));;

	m_triangleContactCapacity = def.triangleContactCapacity;
	m_triangleContactCount = 0;
	m_triangleContacts = (b3ParticleTriangleContact**)m_allocator->Allocate(m_triangleContactCapacity * sizeof(b3ParticleTriangleContact*));;
}

b3ClothSolver::~b3ClothSolver()
{
	m_allocator->Free(m_triangleContacts);
	m_allocator->Free(m_bodyContacts);
	m_allocator->Free(m_forces);
	m_allocator->Free(m_particles);
}

void b3ClothSolver::Add(b3Particle* p)
{
	p->m_solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3ClothSolver::Add(b3Force* f)
{
	m_forces[m_forceCount++] = f;
}

void b3ClothSolver::Add(b3ParticleBodyContact* c)
{
	m_bodyContacts[m_bodyContactCount++] = c;
}

void b3ClothSolver::Add(b3ParticleTriangleContact* c)
{
	m_triangleContacts[m_triangleContactCount++] = c;
}

void b3ClothSolver::Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations)
{
	{
		// Solve internal dynamics
		b3ClothForceSolverDef forceSolverDef;
		forceSolverDef.stack = m_allocator;
		forceSolverDef.particleCount = m_particleCount;
		forceSolverDef.particles = m_particles;
		forceSolverDef.forceCount = m_forceCount;
		forceSolverDef.forces = m_forces;

		b3ClothForceSolver forceSolver(forceSolverDef);

		forceSolver.Solve(dt, gravity);
	}

	// Copy particle state to state buffer
	b3Vec3* positions = (b3Vec3*)m_allocator->Allocate(m_particleCount * sizeof(b3Vec3));
	b3Vec3* velocities = (b3Vec3*)m_allocator->Allocate(m_particleCount * sizeof(b3Vec3));

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		positions[i] = m_particles[i]->m_position;
		velocities[i] = m_particles[i]->m_velocity;
	}

	{
		// Solve constraints
		b3ClothContactSolverDef contactSolverDef;
		contactSolverDef.allocator = m_allocator;
		contactSolverDef.positions = positions;
		contactSolverDef.velocities = velocities;
		contactSolverDef.bodyContactCount = m_bodyContactCount;
		contactSolverDef.bodyContacts = m_bodyContacts;
		contactSolverDef.triangleContactCount = m_triangleContactCount;
		contactSolverDef.triangleContacts = m_triangleContacts;

		b3ClothContactSolver contactSolver(contactSolverDef);

		{
			// Initialize constraints
			contactSolver.InitializeBodyContactConstraints();
			contactSolver.InitializeTriangleContactConstraints();
		}

		{
			// Warm start velocity constraints
			contactSolver.WarmStartBodyContactConstraints();
			contactSolver.WarmStartTriangleContactConstraints();
		}

		{
			// Solve velocity constraints
			for (u32 i = 0; i < velocityIterations; ++i)
			{
				contactSolver.SolveBodyContactVelocityConstraints();
				contactSolver.SolveTriangleContactVelocityConstraints();
			}
		}

		{
			// Cache impulses for warm-starting
			contactSolver.StoreImpulses();
		}

		// Integrate positions
		float32 h = dt;
		for (u32 i = 0; i < m_particleCount; ++i)
		{
			positions[i] += h * velocities[i];
		}

		// Solve position constraints
		{
			bool positionSolved = false;
			for (u32 i = 0; i < positionIterations; ++i)
			{
				bool bodyContactsSolved = contactSolver.SolveBodyContactPositionConstraints();
				bool triangleContactsSolved = contactSolver.SolveTriangleContactPositionConstraints();

				if (bodyContactsSolved && triangleContactsSolved)
				{
					// Early out if the position errors are small.
					positionSolved = true;
					break;
				}
			}
		}

		// Synchronize bodies
		for (u32 i = 0; i < m_bodyContactCount; ++i)
		{
			b3Body* body = m_bodyContacts[i]->m_s2->GetBody();

			body->SynchronizeTransform();

			body->m_worldInvI = b3RotateToFrame(body->m_invI, body->m_xf.rotation);

			body->SynchronizeShapes();
		}
	}

	// Copy state buffers back to the particles
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		p->m_position = positions[i];
		p->m_velocity = velocities[i];
	}

	m_allocator->Free(velocities);
	m_allocator->Free(positions);
}