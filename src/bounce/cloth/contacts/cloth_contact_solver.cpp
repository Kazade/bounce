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

#include <bounce/cloth/contacts/cloth_contact_solver.h>
#include <bounce/cloth/cloth_contact_manager.h>
#include <bounce/cloth/shapes/cloth_sphere_shape.h>
#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/shapes/cloth_world_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>

b3ClothContactSolver::b3ClothContactSolver(const b3ClothContactSolverDef& def)
{
	m_step = def.step;
	m_allocator = def.allocator;
	
	m_positions = def.positions;
	m_velocities = def.velocities;
	
	m_shapeContactCount = def.shapeContactCount;
	m_shapeContacts = def.shapeContacts;
	m_shapeVelocityConstraints = (b3ClothSolverShapeContactVelocityConstraint*)m_allocator->Allocate(m_shapeContactCount * sizeof(b3ClothSolverShapeContactVelocityConstraint));
	m_shapePositionConstraints = (b3ClothSolverShapeContactPositionConstraint*)m_allocator->Allocate(m_shapeContactCount * sizeof(b3ClothSolverShapeContactPositionConstraint));

	m_triangleContactCount = def.triangleContactCount;
	m_triangleContacts = def.triangleContacts;
	m_triangleVelocityConstraints = (b3ClothSolverTriangleContactVelocityConstraint*)m_allocator->Allocate(m_triangleContactCount * sizeof(b3ClothSolverTriangleContactVelocityConstraint));
	m_trianglePositionConstraints = (b3ClothSolverTriangleContactPositionConstraint*)m_allocator->Allocate(m_triangleContactCount * sizeof(b3ClothSolverTriangleContactPositionConstraint));
	
	m_capsuleContactCount = def.capsuleContactCount;
	m_capsuleContacts = def.capsuleContacts;
	m_capsuleVelocityConstraints = (b3ClothSolverCapsuleContactVelocityConstraint*)m_allocator->Allocate(m_capsuleContactCount * sizeof(b3ClothSolverCapsuleContactVelocityConstraint));
	m_capsulePositionConstraints = (b3ClothSolverCapsuleContactPositionConstraint*)m_allocator->Allocate(m_capsuleContactCount * sizeof(b3ClothSolverCapsuleContactPositionConstraint));
}

b3ClothContactSolver::~b3ClothContactSolver()
{
	m_allocator->Free(m_capsulePositionConstraints);
	m_allocator->Free(m_capsuleVelocityConstraints);
	
	m_allocator->Free(m_trianglePositionConstraints);
	m_allocator->Free(m_triangleVelocityConstraints);

	m_allocator->Free(m_shapePositionConstraints);
	m_allocator->Free(m_shapeVelocityConstraints);
}

void b3ClothContactSolver::InitializeShapeContactConstraints()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSphereAndShapeContact* c = m_shapeContacts[i];
		b3ClothSolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;
		b3ClothSolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		b3ClothSphereShape* s1 = c->m_s1;
		b3ClothParticle* p1 = s1->m_p;
		
		const b3Shape* s2 = c->m_s2->m_shape;

		vc->indexA = p1->m_solverId;
		vc->invMassA = p1->m_invMass;
		vc->friction = b3MixFriction(s1->m_friction, s2->GetFriction());

		pc->indexA = p1->m_solverId;
		pc->invMassA = p1->m_invMass;
		pc->radiusA = s1->m_radius;

		pc->radiusB = s2->m_radius;
		
		pc->normal = c->m_normal2;
		pc->pointB = c->m_point2;
	}

	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSphereAndShapeContact* c = m_shapeContacts[i];
		b3ClothSolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;
		b3ClothSolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		u32 indexA = vc->indexA;
		scalar mA = vc->invMassA;
		b3Vec3 cA = m_positions[indexA];

		b3ClothSphereAndShapeContactWorldPoint wp;
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

void b3ClothContactSolver::InitializeTriangleContactConstraints()
{
	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSphereAndTriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		b3ClothSphereShape* s1 = c->m_s1;
		b3ClothParticle* p1 = s1->m_p;
		
		b3ClothTriangleShape* s2 = c->m_s2;
		b3ClothParticle* p2 = s2->m_p1;
		b3ClothParticle* p3 = s2->m_p2;
		b3ClothParticle* p4 = s2->m_p3;

		vc->indexA = p1->m_solverId;
		vc->invMassA = p1->m_invMass;

		vc->indexB = p2->m_solverId;
		vc->invMassB = p2->m_invMass;

		vc->indexC = p3->m_solverId;
		vc->invMassC = p3->m_invMass;

		vc->indexD = p4->m_solverId;
		vc->invMassD = p4->m_invMass;

		pc->indexA = p1->m_solverId;
		pc->invMassA = p1->m_invMass;
		pc->radiusA = s1->m_radius;

		pc->indexB = p2->m_solverId;
		pc->invMassB = p2->m_invMass;

		pc->indexC = p3->m_solverId;
		pc->invMassC = p3->m_invMass;

		pc->indexD = p4->m_solverId;
		pc->invMassD = p4->m_invMass;

		pc->triangleRadius = s2->m_radius;

		pc->wB = c->m_w1;
		pc->wC = c->m_w2;
		pc->wD = c->m_w3;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;

		u32 indexB = pc->indexB;
		scalar mB = pc->invMassB;

		u32 indexC = pc->indexC;
		scalar mC = pc->invMassC;

		u32 indexD = pc->indexD;
		scalar mD = pc->invMassD;

		b3Vec3 xA = m_positions[indexA];
		b3Vec3 xB = m_positions[indexB];
		b3Vec3 xC = m_positions[indexC];
		b3Vec3 xD = m_positions[indexD];

		scalar wB = pc->wB;
		scalar wC = pc->wC;
		scalar wD = pc->wD;

		vc->wB = wB;
		vc->wC = wC;
		vc->wD = wD;

		vc->normal = c->m_normal1;
		vc->normalImpulse = m_step.dt_ratio * c->m_normalImpulse;

		vc->friction = b3MixFriction(s1->m_friction, s2->m_friction);
		vc->tangent1 = c->m_tangent1;
		vc->tangent2 = c->m_tangent2;
		vc->tangentImpulse = m_step.dt_ratio * c->m_tangentImpulse;

		{
			scalar K = mA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

			vc->normalMass = K > scalar(0) ? scalar(1) / K : scalar(0);
		}

		{
			// dot(t1, t2) = 0
			// w * w * dot(t1, t2) = 0
			// J * m * J = m * w * w
			// k22 = k11
			// k21 = k12 = 0
			scalar k11 = mA + mB * wB * wB + mC * wC * wC + mD * wD * wD;
			
			scalar inv_k11 = k11 > scalar(0) ? scalar(1) / k11 : scalar(0);

			b3Mat22 invK;
			invK.x.Set(inv_k11, scalar(0));
			invK.y.Set(scalar(0), inv_k11);

			vc->tangentMass = invK;
		}
	}
}

void b3ClothContactSolver::InitializeCapsuleContactConstraints()
{
	for (u32 i = 0; i < m_capsuleContactCount; ++i)
	{
		b3ClothCapsuleAndCapsuleContact* c = m_capsuleContacts[i];
		b3ClothSolverCapsuleContactVelocityConstraint* vc = m_capsuleVelocityConstraints + i;
		b3ClothSolverCapsuleContactPositionConstraint* pc = m_capsulePositionConstraints + i;

		b3ClothCapsuleShape* s1 = c->m_s1;
		b3ClothParticle* p1 = s1->m_p1;
		b3ClothParticle* p2 = s1->m_p2;

		b3ClothCapsuleShape* s2 = c->m_s2;
		b3ClothParticle* p3 = s2->m_p1;
		b3ClothParticle* p4 = s2->m_p2;

		vc->indexA = p1->m_solverId;
		vc->invMassA = p1->m_invMass;

		vc->indexB = p2->m_solverId;
		vc->invMassB = p2->m_invMass;

		vc->indexC = p3->m_solverId;
		vc->invMassC = p3->m_invMass;

		vc->indexD = p4->m_solverId;
		vc->invMassD = p4->m_invMass;

		pc->indexA = p1->m_solverId;
		pc->invMassA = p1->m_invMass;

		pc->indexB = p2->m_solverId;
		pc->invMassB = p2->m_invMass;
		
		pc->radiusA = s1->m_radius;

		pc->indexC = p3->m_solverId;
		pc->invMassC = p3->m_invMass;

		pc->indexD = p4->m_solverId;
		pc->invMassD = p4->m_invMass;

		pc->radiusB = s2->m_radius;

		pc->wA = c->m_w1;
		pc->wB = c->m_w2;
		pc->wC = c->m_w3;
		pc->wD = c->m_w4;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;

		u32 indexB = pc->indexB;
		scalar mB = pc->invMassB;

		u32 indexC = pc->indexC;
		scalar mC = pc->invMassC;

		u32 indexD = pc->indexD;
		scalar mD = pc->invMassD;

		b3Vec3 xA = m_positions[indexA];
		b3Vec3 xB = m_positions[indexB];
		b3Vec3 xC = m_positions[indexC];
		b3Vec3 xD = m_positions[indexD];

		scalar wA = pc->wA;
		scalar wB = pc->wB;
		scalar wC = pc->wC;
		scalar wD = pc->wD;

		vc->wA = wA;
		vc->wB = wB;
		vc->wC = wC;
		vc->wD = wD;

		vc->normal = c->m_normal1;
		vc->normalImpulse = m_step.dt_ratio * c->m_normalImpulse;

		vc->friction = b3MixFriction(s1->m_friction, s2->m_friction);
		vc->tangent1 = c->m_tangent1;
		vc->tangent2 = c->m_tangent2;
		vc->tangentImpulse = m_step.dt_ratio * c->m_tangentImpulse;

		{
			scalar K = mA * wA * wA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

			vc->normalMass = K > scalar(0) ? scalar(1) / K : scalar(0);
		}

		{
			// dot(t1, t2) = 0
			// w * w * dot(t1, t2) = 0
			// J * m * J = m * w * w
			// k22 = k11
			// k21 = k12 = 0
			scalar k11 = mA * wA * wA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

			scalar inv_k11 = k11 > scalar(0) ? scalar(1) / k11 : scalar(0);

			b3Mat22 invK;
			invK.x.Set(inv_k11, scalar(0));
			invK.y.Set(scalar(0), inv_k11);

			vc->tangentMass = invK;
		}
	}
}

void b3ClothContactSolver::WarmStartShapeContactConstraints()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

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

void b3ClothContactSolver::WarmStartTriangleContactConstraints()
{
	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 indexC = vc->indexC;
		u32 indexD = vc->indexD;

		b3Vec3 vA = m_velocities[indexA];
		b3Vec3 vB = m_velocities[indexB];
		b3Vec3 vC = m_velocities[indexC];
		b3Vec3 vD = m_velocities[indexD];

		scalar mA = vc->invMassA;
		scalar mB = vc->invMassB;
		scalar mC = vc->invMassC;
		scalar mD = vc->invMassD;

		{
			b3Vec3 JA = -vc->normal;
			b3Vec3 JB = vc->wB * vc->normal;
			b3Vec3 JC = vc->wC * vc->normal;
			b3Vec3 JD = vc->wD * vc->normal;

			b3Vec3 PA = vc->normalImpulse * JA;
			b3Vec3 PB = vc->normalImpulse * JB;
			b3Vec3 PC = vc->normalImpulse * JC;
			b3Vec3 PD = vc->normalImpulse * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD; 
		}

		{
			b3Vec3 JA = -vc->tangent1;
			b3Vec3 JB = vc->wB * vc->tangent1;
			b3Vec3 JC = vc->wC * vc->tangent1;
			b3Vec3 JD = vc->wD * vc->tangent1;

			b3Vec3 PA = vc->tangentImpulse.x * JA;
			b3Vec3 PB = vc->tangentImpulse.x * JB;
			b3Vec3 PC = vc->tangentImpulse.x * JC;
			b3Vec3 PD = vc->tangentImpulse.x * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD; 
		}
		
		{
			b3Vec3 JA = -vc->tangent2;
			b3Vec3 JB = vc->wB * vc->tangent2;
			b3Vec3 JC = vc->wC * vc->tangent2;
			b3Vec3 JD = vc->wD * vc->tangent2;

			b3Vec3 PA = vc->tangentImpulse.y * JA;
			b3Vec3 PB = vc->tangentImpulse.y * JB;
			b3Vec3 PC = vc->tangentImpulse.y * JC;
			b3Vec3 PD = vc->tangentImpulse.y * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}
		
		m_velocities[indexA] = vA;
		m_velocities[indexB] = vB;
		m_velocities[indexC] = vC;
		m_velocities[indexD] = vD;
	}
}

void b3ClothContactSolver::WarmStartCapsuleContactConstraints()
{
	for (u32 i = 0; i < m_capsuleContactCount; ++i)
	{
		b3ClothSolverCapsuleContactVelocityConstraint* vc = m_capsuleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 indexC = vc->indexC;
		u32 indexD = vc->indexD;

		b3Vec3 vA = m_velocities[indexA];
		b3Vec3 vB = m_velocities[indexB];
		b3Vec3 vC = m_velocities[indexC];
		b3Vec3 vD = m_velocities[indexD];

		scalar mA = vc->invMassA;
		scalar mB = vc->invMassB;
		scalar mC = vc->invMassC;
		scalar mD = vc->invMassD;

		{
			b3Vec3 JA = -vc->wA * vc->normal;
			b3Vec3 JB = -vc->wB * vc->normal;
			b3Vec3 JC = vc->wC * vc->normal;
			b3Vec3 JD = vc->wD * vc->normal;

			b3Vec3 PA = vc->normalImpulse * JA;
			b3Vec3 PB = vc->normalImpulse * JB;
			b3Vec3 PC = vc->normalImpulse * JC;
			b3Vec3 PD = vc->normalImpulse * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}

		{
			b3Vec3 JA = -vc->wA * vc->tangent1;
			b3Vec3 JB = -vc->wB * vc->tangent1;
			b3Vec3 JC = vc->wC * vc->tangent1;
			b3Vec3 JD = vc->wD * vc->tangent1;

			b3Vec3 PA = vc->tangentImpulse.x * JA;
			b3Vec3 PB = vc->tangentImpulse.x * JB;
			b3Vec3 PC = vc->tangentImpulse.x * JC;
			b3Vec3 PD = vc->tangentImpulse.x * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}

		{
			b3Vec3 JA = -vc->wA * vc->tangent2;
			b3Vec3 JB = -vc->wB * vc->tangent2;
			b3Vec3 JC = vc->wC * vc->tangent2;
			b3Vec3 JD = vc->wD * vc->tangent2;

			b3Vec3 PA = vc->tangentImpulse.y * JA;
			b3Vec3 PB = vc->tangentImpulse.y * JB;
			b3Vec3 PC = vc->tangentImpulse.y * JC;
			b3Vec3 PD = vc->tangentImpulse.y * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}

		m_velocities[indexA] = vA;
		m_velocities[indexB] = vB;
		m_velocities[indexC] = vC;
		m_velocities[indexD] = vD;
	}
}

void b3ClothContactSolver::SolveShapeContactVelocityConstraints()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

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

void b3ClothContactSolver::SolveTriangleContactVelocityConstraints()
{
	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		scalar mA = vc->invMassA;

		u32 indexB = vc->indexB;
		scalar mB = vc->invMassB;

		u32 indexC = vc->indexC;
		scalar mC = vc->invMassC;

		u32 indexD = vc->indexD;
		scalar mD = vc->invMassD;

		scalar wB = vc->wB;
		scalar wC = vc->wC;
		scalar wD = vc->wD;

		b3Vec3 vA = m_velocities[indexA];
		b3Vec3 vB = m_velocities[indexB];
		b3Vec3 vC = m_velocities[indexC];
		b3Vec3 vD = m_velocities[indexD];

		// Solve normal constraint.
		{
			b3Vec3 vCB = wB * vB + wC * vC + wD * vD;

			scalar Cdot = b3Dot(vCB - vA, vc->normal);

			scalar impulse = -vc->normalMass * Cdot;

			scalar oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, scalar(0));
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 JA = -vc->normal;
			b3Vec3 JB = wB * vc->normal;
			b3Vec3 JC = wC * vc->normal;
			b3Vec3 JD = wD * vc->normal;

			b3Vec3 PA = impulse * JA;
			b3Vec3 PB = impulse * JB;
			b3Vec3 PC = impulse * JC;
			b3Vec3 PD = impulse * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}

		// Solve tangent constraints.
		{
			b3Vec3 vCB = wB * vB + wC * vC + wD * vD;

			b3Vec3 dv = vCB - vA;

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

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

			{
				b3Vec3 JA = -vc->tangent1;
				b3Vec3 JB = wB * vc->tangent1;
				b3Vec3 JC = wC * vc->tangent1;
				b3Vec3 JD = wD * vc->tangent1;

				b3Vec3 PA = impulse.x * JA;
				b3Vec3 PB = impulse.x * JB;
				b3Vec3 PC = impulse.x * JC;
				b3Vec3 PD = impulse.x * JD;

				vA += mA * PA;
				vB += mB * PB;
				vC += mC * PC;
				vD += mD * PD;
			}
			
			{
				b3Vec3 JA = -vc->tangent2;
				b3Vec3 JB = wB * vc->tangent2;
				b3Vec3 JC = wC * vc->tangent2;
				b3Vec3 JD = wD * vc->tangent2;

				b3Vec3 PA = impulse.y * JA;
				b3Vec3 PB = impulse.y * JB;
				b3Vec3 PC = impulse.y * JC;
				b3Vec3 PD = impulse.y * JD;

				vA += mA * PA;
				vB += mB * PB;
				vC += mC * PC;
				vD += mD * PD;
			}
		}

		m_velocities[indexA] = vA;
		m_velocities[indexB] = vB;
		m_velocities[indexC] = vC;
		m_velocities[indexD] = vD;
	}
}

void b3ClothContactSolver::SolveCapsuleContactVelocityConstraints()
{
	for (u32 i = 0; i < m_capsuleContactCount; ++i)
	{
		b3ClothSolverCapsuleContactVelocityConstraint* vc = m_capsuleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		scalar mA = vc->invMassA;

		u32 indexB = vc->indexB;
		scalar mB = vc->invMassB;

		u32 indexC = vc->indexC;
		scalar mC = vc->invMassC;

		u32 indexD = vc->indexD;
		scalar mD = vc->invMassD;

		scalar wA = vc->wA;
		scalar wB = vc->wB;
		scalar wC = vc->wC;
		scalar wD = vc->wD;

		b3Vec3 vA = m_velocities[indexA];
		b3Vec3 vB = m_velocities[indexB];
		b3Vec3 vC = m_velocities[indexC];
		b3Vec3 vD = m_velocities[indexD];

		// Solve normal constraint.
		{
			b3Vec3 vCA = wA * vA + wB * vB;
			b3Vec3 vCB = wC * vC + wD * vD;

			scalar Cdot = b3Dot(vCB - vCA, vc->normal);

			scalar impulse = -vc->normalMass * Cdot;

			scalar oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, scalar(0));
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 JA = -wA * vc->normal;
			b3Vec3 JB = -wB * vc->normal;
			b3Vec3 JC = wC * vc->normal;
			b3Vec3 JD = wD * vc->normal;

			b3Vec3 PA = impulse * JA;
			b3Vec3 PB = impulse * JB;
			b3Vec3 PC = impulse * JC;
			b3Vec3 PD = impulse * JD;

			vA += mA * PA;
			vB += mB * PB;
			vC += mC * PC;
			vD += mD * PD;
		}

		// Solve tangent constraints.
		{
			b3Vec3 vCA = wA * vA + wB * vB;
			b3Vec3 vCB = wC * vC + wD * vD;

			b3Vec3 dv = vCB - vCA;

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

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

			{
				b3Vec3 JA = -wA * vc->tangent1;
				b3Vec3 JB = -wB * vc->tangent1;
				b3Vec3 JC = wC * vc->tangent1;
				b3Vec3 JD = wD * vc->tangent1;

				b3Vec3 PA = impulse.x * JA;
				b3Vec3 PB = impulse.x * JB;
				b3Vec3 PC = impulse.x * JC;
				b3Vec3 PD = impulse.x * JD;

				vA += mA * PA;
				vB += mB * PB;
				vC += mC * PC;
				vD += mD * PD;
			}

			{
				b3Vec3 JA = -wA * vc->tangent2;
				b3Vec3 JB = -wB * vc->tangent2;
				b3Vec3 JC = wC * vc->tangent2;
				b3Vec3 JD = wD * vc->tangent2;

				b3Vec3 PA = impulse.y * JA;
				b3Vec3 PB = impulse.y * JB;
				b3Vec3 PC = impulse.y * JC;
				b3Vec3 PD = impulse.y * JD;

				vA += mA * PA;
				vB += mB * PB;
				vC += mC * PC;
				vD += mD * PD;
			}
		}

		m_velocities[indexA] = vA;
		m_velocities[indexB] = vB;
		m_velocities[indexC] = vC;
		m_velocities[indexD] = vD;
	}
}

void b3ClothContactSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSphereAndShapeContact* c = m_shapeContacts[i];
		b3ClothSolverShapeContactVelocityConstraint* vc = m_shapeVelocityConstraints + i;

		c->m_normalImpulse = vc->normalImpulse;
		c->m_tangentImpulse = vc->tangentImpulse;
	}

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSphereAndTriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		c->m_normalImpulse = vc->normalImpulse;
		c->m_tangentImpulse = vc->tangentImpulse;
	}
	
	for (u32 i = 0; i < m_capsuleContactCount; ++i)
	{
		b3ClothCapsuleAndCapsuleContact* c = m_capsuleContacts[i];
		b3ClothSolverCapsuleContactVelocityConstraint* vc = m_capsuleVelocityConstraints + i;

		c->m_normalImpulse = vc->normalImpulse;
		c->m_tangentImpulse = vc->tangentImpulse;
	}
}

struct b3ClothSolverBodyContactSolverPoint
{
	void Initialize(const b3ClothSolverShapeContactPositionConstraint* pc, const b3Vec3 cA)
	{
		b3Vec3 nB = pc->normal;
		b3Vec3 cB = pc->pointB;

		scalar rA = pc->radiusA;
		scalar rB = pc->radiusB;
		
		b3Vec3 pA = cA - rA * nB;
		b3Vec3 pB = cB + rB * nB;

		point = cB;
		normal = nB;
		separation = b3Dot(cA - cB, nB) - rA - rB;
	}

	b3Vec3 normal;
	b3Vec3 point;
	scalar separation;
};

bool b3ClothContactSolver::SolveShapeContactPositionConstraints()
{
	scalar minSeparation(0);

	for (u32 i = 0; i < m_shapeContactCount; ++i)
	{
		b3ClothSolverShapeContactPositionConstraint* pc = m_shapePositionConstraints + i;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;

		b3Vec3 cA = m_positions[indexA];

		// Solve normal constraint
		b3ClothSolverBodyContactSolverPoint cpcp;
		cpcp.Initialize(pc, cA);

		b3Vec3 normal = cpcp.normal;
		b3Vec3 point = cpcp.point;
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

	return minSeparation >= scalar(-3.0) * B3_LINEAR_SLOP;
}

bool b3ClothContactSolver::SolveTriangleContactPositionConstraints()
{
	scalar minSeparation(0);

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;
		scalar radiusA = pc->radiusA;

		u32 indexB = pc->indexB;
		scalar mB = pc->invMassB;

		u32 indexC = pc->indexC;
		scalar mC = pc->invMassC;

		u32 indexD = pc->indexD;
		scalar mD = pc->invMassD;

		scalar triangleRadius = pc->triangleRadius;

		scalar wB = pc->wB;
		scalar wC = pc->wC;
		scalar wD = pc->wD;

		b3Vec3 xA = m_positions[indexA];
		b3Vec3 xB = m_positions[indexB];
		b3Vec3 xC = m_positions[indexC];
		b3Vec3 xD = m_positions[indexD];

		b3Vec3 cB = wB * xB + wC * xC + wD * xD;

		b3Vec3 n = cB - xA;
		
		scalar distance = n.Normalize();

		scalar separation = distance - radiusA - triangleRadius;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		scalar C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, scalar(0));

		b3Vec3 JA = -n;
		b3Vec3 JB = wB * n;
		b3Vec3 JC = wC * n;
		b3Vec3 JD = wD * n;

		// Compute effective mass.
		scalar K = mA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

		// Compute normal impulse.
		scalar impulse = K > scalar(0) ? -C / K : scalar(0);

		b3Vec3 PA = impulse * JA;
		b3Vec3 PB = impulse * JB;
		b3Vec3 PC = impulse * JC;
		b3Vec3 PD = impulse * JD;

		xA += mA * PA;
		xB += mB * PB;
		xC += mC * PC;
		xD += mD * PD;

		m_positions[indexA] = xA;
		m_positions[indexB] = xB;
		m_positions[indexC] = xC;
		m_positions[indexD] = xD;
	}

	return minSeparation >= scalar(-3.0) * B3_LINEAR_SLOP;
}

bool b3ClothContactSolver::SolveCapsuleContactPositionConstraints()
{
	scalar minSeparation(0);

	for (u32 i = 0; i < m_capsuleContactCount; ++i)
	{
		b3ClothSolverCapsuleContactPositionConstraint* pc = m_capsulePositionConstraints + i;

		u32 indexA = pc->indexA;
		scalar mA = pc->invMassA;

		u32 indexB = pc->indexB;
		scalar mB = pc->invMassB;

		scalar radiusA = pc->radiusA;
		
		u32 indexC = pc->indexC;
		scalar mC = pc->invMassC;

		u32 indexD = pc->indexD;
		scalar mD = pc->invMassD;

		scalar radiusB = pc->radiusB;

		scalar wA = pc->wA;
		scalar wB = pc->wB;
		scalar wC = pc->wC;
		scalar wD = pc->wD;

		b3Vec3 xA = m_positions[indexA];
		b3Vec3 xB = m_positions[indexB];
		b3Vec3 xC = m_positions[indexC];
		b3Vec3 xD = m_positions[indexD];

		b3Vec3 cA = wA * xA + wB * xB;
		b3Vec3 cB = wC * xC + wD * xD;

		b3Vec3 n = cB - cA;

		scalar distance = n.Normalize();

		scalar separation = distance - radiusA - radiusB;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		scalar C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, scalar(0));

		b3Vec3 JA = -wA * n;
		b3Vec3 JB = -wB * n;
		b3Vec3 JC = wC * n;
		b3Vec3 JD = wD * n;

		// Compute effective mass.
		scalar K = mA * wA * wA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

		// Compute normal impulse.
		scalar impulse = K > scalar(0) ? -C / K : scalar(0);

		b3Vec3 PA = impulse * JA;
		b3Vec3 PB = impulse * JB;
		b3Vec3 PC = impulse * JC;
		b3Vec3 PD = impulse * JD;

		xA += mA * PA;
		xB += mB * PB;
		xC += mC * PC;
		xD += mD * PD;

		m_positions[indexA] = xA;
		m_positions[indexB] = xB;
		m_positions[indexC] = xC;
		m_positions[indexD] = xD;
	}

	return minSeparation >= scalar(-3.0) * B3_LINEAR_SLOP;
}