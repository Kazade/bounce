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

#include <bounce/softbody/contacts/softbody_contact_solver.h>
#include <bounce/softbody/contacts/softbody_sphere_shape_contact.h>
#include <bounce/softbody/shapes/softbody_sphere_shape.h>
#include <bounce/softbody/shapes/softbody_world_shape.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>

b3SoftBodyContactSolver::b3SoftBodyContactSolver(const b3SoftBodyContactSolverDef& def)
{
	m_step = def.step;
	m_allocator = def.allocator;

	m_positions = def.positions;
	m_velocities = def.velocities;

	m_shapeContactCount = def.shapeContactCount;
	m_shapeContacts = def.shapeContacts;
}

b3SoftBodyContactSolver::~b3SoftBodyContactSolver()
{
	m_allocator->Free(m_shapePositionConstraints);
	m_allocator->Free(m_shapeVelocityConstraints);
}

void b3SoftBodyContactSolver::InitializeShapeContactConstraints()
{
	m_shapeVelocityConstraints = (b3SoftBodySolverShapeContactVelocityConstraint*)m_allocator->Allocate(m_shapeContactCount * sizeof(b3SoftBodySolverShapeContactVelocityConstraint));
	m_shapePositionConstraints = (b3SoftBodySolverShapeContactPositionConstraint*)m_allocator->Allocate(m_shapeContactCount * sizeof(b3SoftBodySolverShapeContactPositionConstraint));

	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySphereAndShapeContact* c = m_shapeContacts[i];
		b3SoftBodySolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;
		b3SoftBodySolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		b3SoftBodySphereShape* s1 = c->m_s1;
		b3SoftBodyNode* n1 = s1->m_node;

		b3SoftBodyWorldShape* ws2 = c->m_s2;
		const b3Shape* s2 = ws2->m_shape;

		vc->indexA = n1->m_meshIndex;
		vc->invMassA = n1->m_type == e_staticSoftBodyNode ? scalar(0) : n1->m_invMass;

		vc->friction = b3MixFriction(s1->m_friction, s2->GetFriction());

		pc->indexA = n1->m_meshIndex;
		pc->invMassA = n1->m_type == e_staticSoftBodyNode ? scalar(0) : n1->m_invMass;

		pc->radiusA = s1->m_radius;
		pc->radiusB = s2->m_radius;

		pc->normalB = c->m_normal2;
		pc->pointB = c->m_point2;
	}

	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySphereAndShapeContact* c = m_shapeContacts[i];
		b3SoftBodySolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;
		b3SoftBodySolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		u32 indexA = vc->indexA;
		scalar mA = vc->invMassA;
		b3Vec3 cA = m_positions[indexA];

		b3SoftBodySphereAndShapeContactWorldPoint wp;
		wp.Initialize(c, pc->radiusA, cA, pc->radiusB);

		vc->normal = wp.normal;
		vc->tangent1 = c->m_tangent1;
		vc->tangent2 = c->m_tangent2;

		vc->normalImpulse = m_step.dt_ratio * c->m_normalImpulse;
		vc->tangentImpulse = m_step.dt_ratio * c->m_tangentImpulse;

		{
			vc->normalMass = mA > scalar(0) ? scalar(1) / mA : scalar(0);
		}

		{
			vc->tangentMass = mA > scalar(0) ? scalar(1) / mA : scalar(0);
		}
	}
}

void b3SoftBodyContactSolver::WarmStart()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Vec3 vA = m_velocities[indexA];
		scalar mA = vc->invMassA;

		b3Vec3 P = vc->normalImpulse * vc->normal;

		vA += mA * P;

		b3Vec3 P1 = vc->tangentImpulse.x * vc->tangent1;
		b3Vec3 P2 = vc->tangentImpulse.y * vc->tangent2;

		vA += mA * (P1 + P2);

		m_velocities[indexA] = vA;
	}
}

void b3SoftBodyContactSolver::SolveShapeContactVelocityConstraints()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Vec3 vA = m_velocities[indexA];
		scalar mA = vc->invMassA;

		b3Vec3 normal = vc->normal;

		// Solve normal constraint.
		{
			scalar Cdot = b3Dot(normal, vA);

			scalar impulse = -vc->normalMass * Cdot;

			scalar oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, scalar(0));
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 P = impulse * normal;

			vA += mA * P;
		}

		// Solve tangent constraints.
		{
			b3Vec2 Cdot;
			Cdot.x = b3Dot(vA, vc->tangent1);
			Cdot.y = b3Dot(vA, vc->tangent2);

			b3Vec2 impulse = vc->tangentMass * -Cdot;
			b3Vec2 oldImpulse = vc->tangentImpulse;
			vc->tangentImpulse += impulse;

			scalar maxImpulse = vc->friction * vc->normalImpulse;
			if (b3Dot(vc->tangentImpulse, vc->tangentImpulse) > maxImpulse * maxImpulse)
			{
				vc->tangentImpulse.Normalize();
				vc->tangentImpulse *= maxImpulse;
			}

			impulse = vc->tangentImpulse - oldImpulse;

			b3Vec3 P1 = impulse.x * vc->tangent1;
			b3Vec3 P2 = impulse.y * vc->tangent2;
			
			b3Vec3 P = P1 + P2;

			vA += mA * P;
		}

		m_velocities[indexA] = vA;
	}
}

void b3SoftBodyContactSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySphereAndShapeContact* c = m_shapeContacts[i];
		b3SoftBodySolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

		c->m_normalImpulse = vc->normalImpulse;
		c->m_tangentImpulse = vc->tangentImpulse;
	}
}

struct b3SoftBodySolverBodyContactSolverPoint
{
	void Initialize(const b3SoftBodySolverShapeContactPositionConstraint* pc, const b3Vec3& cA)
	{
		b3Vec3 nB = pc->normalB;
		b3Vec3 cB = pc->pointB;

		scalar rA = pc->radiusA;
		scalar rB = pc->radiusB;

		normal = nB;
		separation = b3Dot(cA - cB, nB) - rA - rB;
	}

	b3Vec3 normal;
	scalar separation;
};

bool b3SoftBodyContactSolver::SolveShapeContactPositionConstraints()
{
	scalar minSeparation = scalar(0);

	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3SoftBodySolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;
		b3Vec3 cA = m_positions[indexA];

		// Solve normal constraint
		b3SoftBodySolverBodyContactSolverPoint cpcp;
		cpcp.Initialize(pc, cA);

		b3Vec3 normal = cpcp.normal;
		scalar separation = cpcp.separation;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		scalar C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, scalar(0));

		// Compute normal impulse.
		scalar impulse = mA > scalar(0) ? -C / mA : scalar(0);
		
		b3Vec3 P = impulse * normal;

		cA += mA * P;

		m_positions[indexA] = cA;
	}

	return minSeparation >= scalar(-3) * B3_LINEAR_SLOP;
}