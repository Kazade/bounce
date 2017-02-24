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

#include <bounce/dynamics/island.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/joints/joint_solver.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/contact_solver.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/profiler.h>

b3Island::b3Island(b3StackAllocator* allocator, u32 bodyCapacity, u32 contactCapacity, u32 jointCapacity) 
{
	m_allocator = allocator;
	m_bodyCapacity = bodyCapacity;
	m_contactCapacity = contactCapacity;
	m_jointCapacity = jointCapacity;
	
	m_bodies = (b3Body**)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Body*));
	m_velocities = (b3Velocity*)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Velocity));
	m_positions = (b3Position*)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Position));
	m_contacts = (b3Contact**)m_allocator->Allocate(m_contactCapacity * sizeof(b3Contact*));
	m_joints = (b3Joint**)m_allocator->Allocate(m_jointCapacity * sizeof(b3Joint*));

	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;
}

b3Island::~b3Island() 
{
	// @note Reverse order of construction.
	m_allocator->Free(m_joints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_positions);
	m_allocator->Free(m_velocities);
	m_allocator->Free(m_bodies);
}

void b3Island::Clear() 
{
	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;
}

void b3Island::Add(b3Body* b) 
{
	B3_ASSERT(m_bodyCount < m_bodyCapacity);
	b->m_islandID = m_bodyCount;
	m_bodies[m_bodyCount] = b;
	++m_bodyCount;
}

void b3Island::Add(b3Contact* c) 
{
	B3_ASSERT(m_contactCount < m_contactCapacity);
	m_contacts[m_contactCount] = c;
	++m_contactCount;
}

void b3Island::Add(b3Joint* j) 
{
	B3_ASSERT(m_jointCount < m_jointCapacity);
	m_joints[m_jointCount] = j;
	++m_jointCount;
}

void b3Island::Solve(const b3Vec3& gravity, float32 dt, u32 velocityIterations, u32 positionIterations, u32 flags)
{
	float32 h = dt;

	// Measure coefficient of damping.
	// Box2D.
	// ODE: dv/dt + c * v = 0
	// Solution: v(t) = v0 * exp(-c * t)
	// Step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
	// v2 = exp(-c * dt) * v1
	const float32 k_d = 0.1f;
	float32 d = exp(-h * k_d);

	// 1. Integrate velocities
	for (u32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Body* b = m_bodies[i];

		b3Vec3 v = b->m_linearVelocity;
		b3Vec3 w = b->m_angularVelocity;
		b3Vec3 x = b->m_sweep.worldCenter;
		b3Quat q = b->m_sweep.orientation;

		// Remember the positions for CCD
		b->m_sweep.worldCenter0 = b->m_sweep.worldCenter;
		b->m_sweep.orientation0 = b->m_sweep.orientation;

		if (b->m_type == e_dynamicBody) 
		{
			b3Vec3 force = b->m_force + b->m_gravityScale * gravity;
			
			// Integrate forces
			v += h * b->m_invMass * force;
			// Clear forces
			b->m_force.SetZero();
			
			// Integrate torques
			// @todo add gyroscopic term
			w += h * b3Mul(b->m_worldInvI, b->m_torque);
			// Clear torques
			b->m_torque.SetZero();
			
			// Apply damping
			v *= d;
			w *= d;
		}

		m_velocities[i].v = v;
		m_velocities[i].w = w;
		m_positions[i].x = x;
		m_positions[i].q = q;
	}

	b3JointSolverDef jointSolverDef;
	jointSolverDef.joints = m_joints;
	jointSolverDef.count = m_jointCount;
	jointSolverDef.positions = m_positions;
	jointSolverDef.velocities = m_velocities;
	jointSolverDef.dt = h;
	b3JointSolver jointSolver(&jointSolverDef);

	b3ContactSolverDef contactSolverDef;
	contactSolverDef.allocator = m_allocator;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	contactSolverDef.dt = h;
	b3ContactSolver contactSolver(&contactSolverDef);

	// 2. Initialize constraints
	{
		B3_PROFILE("Initialize Constraints");
		
		contactSolver.InitializeConstraints();

		if (flags & e_warmStartBit)
		{
			contactSolver.WarmStart();
		}

		jointSolver.InitializeConstraints();

		if (flags & e_warmStartBit)
		{
			jointSolver.WarmStart();
		}
	}

	// 3. Solve velocity constraints
	{
		B3_PROFILE("Solve Velocity Constraints");

		for (u32 i = 0; i < velocityIterations; ++i)
		{
			jointSolver.SolveVelocityConstraints();
			contactSolver.SolveVelocityConstraints();
		}

		if (flags & e_warmStartBit)
		{
			contactSolver.StoreImpulses();
		}
	}

	// 4. Integrate positions
	for (u32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Vec3 x = m_positions[i].x;
		b3Quat q = m_positions[i].q;
		b3Vec3 v = m_velocities[i].v;
		b3Vec3 w = m_velocities[i].w;

		// Prevent numerical instability due to large velocity changes.		
		b3Vec3 translation = h * v;
		if (b3Dot(translation, translation) > B3_MAX_TRANSLATION_SQUARED)
		{
			float32 ratio = B3_MAX_TRANSLATION / b3Length(translation);
			v *= ratio;
		}

		b3Vec3 rotation = h * w;
		if (b3Dot(rotation, rotation) > B3_MAX_ROTATION_SQUARED)
		{
			float32 ratio = B3_MAX_ROTATION / b3Length(rotation);
			w *= ratio;
		}

		// Integrate position 
		x += h * v;
		// Integrate orientation
		q = b3Integrate(q, w, h);

		m_positions[i].x = x;
		m_positions[i].q = q;
		m_velocities[i].v = v;
		m_velocities[i].w = w;
	}

	// 5. Solve position constraints
	{
		B3_PROFILE("Solve Position Constraints");
		
		bool positionsSolved = false;
		for (u32 i = 0; i < positionIterations; ++i) 
		{
			bool contactsSolved = contactSolver.SolvePositionConstraints();
			bool jointsSolved = jointSolver.SolvePositionConstraints();
			if (contactsSolved && jointsSolved)
			{
				// Early out if the position errors are small.
				positionsSolved = true;
				break;
			}
		}
	}

	// 6. Copy state buffers back to the bodies
	for (u32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Body* b = m_bodies[i];
		b->m_sweep.worldCenter = m_positions[i].x;
		b->m_sweep.orientation = m_positions[i].q;
		b->m_sweep.orientation.Normalize();
		b->m_linearVelocity = m_velocities[i].v;
		b->m_angularVelocity = m_velocities[i].w;	
		b->SynchronizeTransform();
		// Transform body inertia to world inertia
		b->m_worldInvI = b3RotateToFrame(b->m_invI, b->m_xf.rotation);
	}

	// 7. Put bodies under unconsiderable motion to sleep
	if (flags & e_sleepBit) 
	{
		float32 minSleepTime = B3_MAX_FLOAT;
		for (u32 i = 0; i < m_bodyCount; ++i) 
		{
			b3Body* b = m_bodies[i];
			if (b->m_type == e_staticBody) 
			{
				continue;
			}

			// Compute the linear and angular speed of the body.
			float32 sqrLinVel = b3Dot(b->m_linearVelocity, b->m_linearVelocity);
			float32 sqrAngVel = b3Dot(b->m_angularVelocity, b->m_angularVelocity);

			if (sqrLinVel > B3_SLEEP_LINEAR_TOL || sqrAngVel > B3_SLEEP_ANGULAR_TOL) 
			{
				minSleepTime = 0.0f;
				b->m_sleepTime = 0.0f;
			}
			else 
			{
				b->m_sleepTime += h;
				minSleepTime = b3Min(minSleepTime, b->m_sleepTime);
			}
		}

		// Put the island to sleep so long as the minimum found sleep time
		// is below the threshold. 
		if (minSleepTime >= B3_TIME_TO_SLEEP) 
		{
			for (u32 i = 0; i < m_bodyCount; ++i) 
			{
				m_bodies[i]->SetAwake(false);
			}
		}
	}
}
