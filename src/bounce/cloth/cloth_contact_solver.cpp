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

#include <bounce/cloth/cloth_contact_solver.h>
#include <bounce/cloth/cloth_contact_manager.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/dense_vec3.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3ClothContactSolver::b3ClothContactSolver(const b3ClothContactSolverDef& def)
{
	m_allocator = def.allocator;
	
	m_positions = def.positions;
	m_velocities = def.velocities;
	
	m_bodyContactCount = def.bodyContactCount;
	m_bodyContacts = def.bodyContacts;
	m_bodyVelocityConstraints = (b3ClothSolverBodyContactVelocityConstraint*)m_allocator->Allocate(m_bodyContactCount * sizeof(b3ClothSolverBodyContactVelocityConstraint));
	m_bodyPositionConstraints = (b3ClothSolverBodyContactPositionConstraint*)m_allocator->Allocate(m_bodyContactCount * sizeof(b3ClothSolverBodyContactPositionConstraint));

	m_triangleContactCount = def.triangleContactCount;
	m_triangleContacts = def.triangleContacts;
	m_triangleVelocityConstraints = (b3ClothSolverTriangleContactVelocityConstraint*)m_allocator->Allocate(m_triangleContactCount * sizeof(b3ClothSolverTriangleContactVelocityConstraint));
	m_trianglePositionConstraints = (b3ClothSolverTriangleContactPositionConstraint*)m_allocator->Allocate(m_triangleContactCount * sizeof(b3ClothSolverTriangleContactPositionConstraint));
}

b3ClothContactSolver::~b3ClothContactSolver()
{
	m_allocator->Free(m_trianglePositionConstraints);
	m_allocator->Free(m_triangleVelocityConstraints);

	m_allocator->Free(m_bodyPositionConstraints);
	m_allocator->Free(m_bodyVelocityConstraints);
}

void b3ClothContactSolver::InitializeBodyContactConstraints()
{
	b3DenseVec3& x = *m_positions;
	b3DenseVec3& v = *m_velocities;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ParticleBodyContact* c = m_bodyContacts[i];
		b3ClothSolverBodyContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;
		b3ClothSolverBodyContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		vc->indexA = c->p1->m_solverId;
		vc->bodyB = c->s2->GetBody();

		vc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		vc->invMassB = vc->bodyB->GetInverseMass();

		vc->invIA.SetZero();
		vc->invIB = vc->bodyB->GetWorldInverseInertia();

		vc->friction = b3MixFriction(c->p1->m_friction, c->s2->GetFriction());

		pc->indexA = c->p1->m_solverId;
		pc->bodyB = vc->bodyB;

		pc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		pc->invMassB = vc->bodyB->m_invMass;

		pc->invIA.SetZero();
		pc->invIB = vc->bodyB->m_worldInvI;

		pc->radiusA = c->p1->m_radius;
		pc->radiusB = c->s2->m_radius;

		pc->localCenterA.SetZero();
		pc->localCenterB = pc->bodyB->m_sweep.localCenter;

		pc->normalA = c->normal1;
		pc->localPointA = c->localPoint1;
		pc->localPointB = c->localPoint2;
	}

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ParticleBodyContact* c = m_bodyContacts[i];
		b3ClothSolverBodyContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;
		b3ClothSolverBodyContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = bodyB->m_sweep.worldCenter;

		b3Quat qA; qA.SetIdentity();
		b3Quat qB = bodyB->m_sweep.orientation;

		b3Vec3 localCenterA = pc->localCenterA;
		b3Vec3 localCenterB = pc->localCenterB;

		b3Transform xfA;
		xfA.rotation = b3QuatMat33(qA);
		xfA.position = xA - b3Mul(xfA.rotation, localCenterA);

		b3Transform xfB;
		xfB.rotation = b3QuatMat33(qB);
		xfB.position = xB - b3Mul(xfB.rotation, localCenterB);

		b3ParticleBodyContactWorldPoint wp;
		wp.Initialize(c, pc->radiusA, xfA, pc->radiusB, xfB);

		vc->normal = wp.normal;
		vc->tangent1 = c->t1;
		vc->tangent2 = c->t2;
		vc->point = wp.point;

		b3Vec3 point = vc->point;

		b3Vec3 rA = point - xA;
		b3Vec3 rB = point - xB;

		vc->rA = rA;
		vc->rB = rB;

		vc->normalImpulse = c->normalImpulse;
		vc->tangentImpulse = c->tangentImpulse;

		{
			b3Vec3 n = vc->normal;

			b3Vec3 rnA = b3Cross(rA, n);
			b3Vec3 rnB = b3Cross(rB, n);

			float32 K = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);

			vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;
			vc->velocityBias = 0.0f;
		}

		{
			b3Vec3 t1 = vc->tangent1;
			b3Vec3 t2 = vc->tangent2;

			b3Vec3 rn1A = b3Cross(rA, t1);
			b3Vec3 rn1B = b3Cross(rB, t1);
			b3Vec3 rn2A = b3Cross(rA, t2);
			b3Vec3 rn2B = b3Cross(rB, t2);

			// dot(t1, t2) = 0
			// J1_l1 * M1 * J2_l1 = J1_l2 * M2 * J2_l2 = 0
			float32 k11 = mA + mB + b3Dot(iA * rn1A, rn1A) + b3Dot(iB * rn1B, rn1B);
			float32 k12 = b3Dot(iA * rn1A, rn2A) + b3Dot(iB * rn1B, rn2B);
			float32 k22 = mA + mB + b3Dot(iA * rn2A, rn2A) + b3Dot(iB * rn2B, rn2B);

			b3Mat22 K;
			K.x.Set(k11, k12);
			K.y.Set(k12, k22);

			vc->tangentMass = b3Inverse(K);
		}
	}
}
void b3ClothContactSolver::InitializeTriangleContactConstraints()
{
	b3DenseVec3& x = *m_positions;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ParticleTriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		vc->indexA = c->m_p1->m_solverId;
		vc->invMassA = c->m_p1->m_type == e_staticParticle ? 0.0f : c->m_p1->m_invMass;

		vc->indexB = c->m_p2->m_solverId;
		vc->invMassB = c->m_p2->m_type == e_staticParticle ? 0.0f : c->m_p2->m_invMass;

		vc->indexC = c->m_p3->m_solverId;
		vc->invMassC = c->m_p3->m_type == e_staticParticle ? 0.0f : c->m_p3->m_invMass;

		vc->indexD = c->m_p4->m_solverId;
		vc->invMassD = c->m_p4->m_type == e_staticParticle ? 0.0f : c->m_p4->m_invMass;

		pc->indexA = c->m_p1->m_solverId;
		pc->invMassA = c->m_p1->m_type == e_staticParticle ? 0.0f : c->m_p1->m_invMass;
		pc->radiusA = c->m_p1->m_radius;

		pc->indexB = c->m_p2->m_solverId;
		pc->invMassB = c->m_p2->m_type == e_staticParticle ? 0.0f : c->m_p2->m_invMass;

		pc->indexC = c->m_p3->m_solverId;
		pc->invMassC = c->m_p3->m_type == e_staticParticle ? 0.0f : c->m_p3->m_invMass;

		pc->indexD = c->m_p4->m_solverId;
		pc->invMassD = c->m_p4->m_type == e_staticParticle ? 0.0f : c->m_p4->m_invMass;

		pc->triangleRadius = 0.0f;

		pc->wB = c->w2;
		pc->wC = c->w3;
		pc->wD = c->w4;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;

		u32 indexC = pc->indexC;
		float32 mC = pc->invMassC;

		u32 indexD = pc->indexD;
		float32 mD = pc->invMassD;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];
		b3Vec3 xC = x[indexC];
		b3Vec3 xD = x[indexD];

		float32 wB = pc->wB;
		float32 wC = pc->wC;
		float32 wD = pc->wD;

		b3Vec3 cB = wB * xB + wC * xC + wD * xD;

		b3Vec3 n = cB - xA;
		n.Normalize();

		vc->normal = n;
		vc->wB = wB;
		vc->wC = wC;
		vc->wD = wD;

		float32 K = mA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

		vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;
		vc->normalImpulse = c->m_normalImpulse;
	}
}

void b3ClothContactSolver::WarmStartBodyContactConstraints()
{
	b3DenseVec3& v = *m_velocities;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverBodyContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 P = vc->normalImpulse * vc->normal;

		vA -= mA * P;
		wA -= iA * b3Cross(vc->rA, P);

		vB += mB * P;
		wB += iB * b3Cross(vc->rB, P);

		b3Vec3 P1 = vc->tangentImpulse.x * vc->tangent1;
		b3Vec3 P2 = vc->tangentImpulse.y * vc->tangent2;

		vA -= mA * (P1 + P2);
		wA -= iA * b3Cross(vc->rA, P1 + P2);

		vB += mB * (P1 + P2);
		wB += iB * b3Cross(vc->rB, P1 + P2);

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}
}

void b3ClothContactSolver::WarmStartTriangleContactConstraints()
{
	b3DenseVec3& v = *m_velocities;
	
	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 indexC = vc->indexC;
		u32 indexD = vc->indexD;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];
		b3Vec3 vC = v[indexC];
		b3Vec3 vD = v[indexD];

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;
		float32 mC = vc->invMassC;
		float32 mD = vc->invMassD;

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

		v[indexA] = vA;
		v[indexB] = vB;
		v[indexC] = vC;
		v[indexD] = vD;
	}
}

void b3ClothContactSolver::SolveBodyContactVelocityConstraints()
{
	b3DenseVec3& v = *m_velocities;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverBodyContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 normal = vc->normal;
		b3Vec3 point = vc->point;

		b3Vec3 rA = vc->rA;
		b3Vec3 rB = vc->rB;

		// Solve normal constraint.
		{
			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);
			float32 Cdot = b3Dot(normal, dv);

			float32 impulse = vc->normalMass * (-Cdot + vc->velocityBias);

			float32 oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 P = impulse * normal;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		// Solve tangent constraints.
		{
			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

			b3Vec2 impulse = vc->tangentMass * -Cdot;
			b3Vec2 oldImpulse = vc->tangentImpulse;
			vc->tangentImpulse += impulse;

			float32 maxImpulse = vc->friction * vc->normalImpulse;
			if (b3Dot(vc->tangentImpulse, vc->tangentImpulse) > maxImpulse * maxImpulse)
			{
				vc->tangentImpulse.Normalize();
				vc->tangentImpulse *= maxImpulse;
			}

			impulse = vc->tangentImpulse - oldImpulse;

			b3Vec3 P1 = impulse.x * vc->tangent1;
			b3Vec3 P2 = impulse.y * vc->tangent2;
			b3Vec3 P = P1 + P2;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}
}

void b3ClothContactSolver::SolveTriangleContactVelocityConstraints()
{
	b3DenseVec3& v = *m_velocities;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		float32 mA = vc->invMassA;

		u32 indexB = vc->indexB;
		float32 mB = vc->invMassB;

		u32 indexC = vc->indexC;
		float32 mC = vc->invMassC;

		u32 indexD = vc->indexD;
		float32 mD = vc->invMassD;

		float32 wB = vc->wB;
		float32 wC = vc->wC;
		float32 wD = vc->wD;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];
		b3Vec3 vC = v[indexC];
		b3Vec3 vD = v[indexD];

		b3Vec3 vCB = wB * vB + wC * vC + wD * vD;

		float32 Cdot = b3Dot(vCB - vA, vc->normal);

		float32 impulse = -vc->normalMass * Cdot;

		float32 oldImpulse = vc->normalImpulse;
		vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
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

		v[indexA] = vA;
		v[indexB] = vB;
		v[indexC] = vC;
		v[indexD] = vD;
	}
}

void b3ClothContactSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ParticleBodyContact* c = m_bodyContacts[i];
		b3ClothSolverBodyContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		c->normalImpulse = vc->normalImpulse;
		c->tangentImpulse = vc->tangentImpulse;
	}

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ParticleTriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		c->m_normalImpulse = vc->normalImpulse;
	}
}

struct b3ClothSolverBodyContactSolverPoint
{
	void Initialize(const b3ClothSolverBodyContactPositionConstraint* pc, const b3Transform& xfA, const b3Transform& xfB)
	{
		b3Vec3 nA = pc->normalA;

		b3Vec3 cA = b3Mul(xfA, pc->localPointA);
		b3Vec3 cB = b3Mul(xfB, pc->localPointB);

		float32 rA = pc->radiusA;
		float32 rB = pc->radiusB;
		
		b3Vec3 pA = cA + rA * nA;
		b3Vec3 pB = cB - rB * nA;

		point = cB;
		normal = nA;
		separation = b3Dot(cB - cA, nA) - rA - rB;
	}

	b3Vec3 normal;
	b3Vec3 point;
	float32 separation;
};

bool b3ClothContactSolver::SolveBodyContactPositionConstraints()
{
	b3DenseVec3& x = *m_positions;

	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverBodyContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		b3Mat33 iA = pc->invIA;
		b3Vec3 localCenterA = pc->localCenterA;

		b3Body* bodyB = pc->bodyB;
		float32 mB = pc->invMassB;
		b3Mat33 iB = pc->invIB;
		b3Vec3 localCenterB = pc->localCenterB;

		b3Vec3 cA = x[indexA];
		b3Quat qA; qA.SetIdentity();

		b3Vec3 cB = bodyB->m_sweep.worldCenter;
		b3Quat qB = bodyB->m_sweep.orientation;

		// Solve normal constraint
		b3Transform xfA;
		xfA.rotation = b3QuatMat33(qA);
		xfA.position = cA - b3Mul(xfA.rotation, localCenterA);

		b3Transform xfB;
		xfB.rotation = b3QuatMat33(qB);
		xfB.position = cB - b3Mul(xfB.rotation, localCenterB);

		b3ClothSolverBodyContactSolverPoint cpcp;
		cpcp.Initialize(pc, xfA, xfB);

		b3Vec3 normal = cpcp.normal;
		b3Vec3 point = cpcp.point;
		float32 separation = cpcp.separation;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);

		// Compute effective mass.
		b3Vec3 rA = point - cA;
		b3Vec3 rB = point - cB;

		b3Vec3 rnA = b3Cross(rA, normal);
		b3Vec3 rnB = b3Cross(rB, normal);
		float32 K = mA + mB + b3Dot(rnA, iA * rnA) + b3Dot(rnB, iB * rnB);

		// Compute normal impulse.
		float32 impulse = K > 0.0f ? -C / K : 0.0f;
		b3Vec3 P = impulse * normal;

		cA -= mA * P;
		qA -= b3Derivative(qA, iA * b3Cross(rA, P));
		qA.Normalize();

		cB += mB * P;
		qB += b3Derivative(qB, iB * b3Cross(rB, P));
		qB.Normalize();

		x[indexA] = cA;

		bodyB->m_sweep.worldCenter = cB;
		bodyB->m_sweep.orientation = qB;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}

bool b3ClothContactSolver::SolveTriangleContactPositionConstraints()
{
	b3DenseVec3& x = *m_positions;

	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		float32 radiusA = pc->radiusA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;

		u32 indexC = pc->indexC;
		float32 mC = pc->invMassC;

		u32 indexD = pc->indexD;
		float32 mD = pc->invMassD;

		float32 triangleRadius = pc->triangleRadius;

		float32 wB = pc->wB;
		float32 wC = pc->wC;
		float32 wD = pc->wD;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];
		b3Vec3 xC = x[indexC];
		b3Vec3 xD = x[indexD];

		b3Vec3 cB = wB * xB + wC * xC + wD * xD;

		b3Vec3 n = cB - xA;
		
		float32 distance = n.Normalize();

		float32 separation = distance - radiusA - triangleRadius;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);

		b3Vec3 JA = -n;
		b3Vec3 JB = wB * n;
		b3Vec3 JC = wC * n;
		b3Vec3 JD = wD * n;

		// Compute effective mass.
		float32 K = mA + mB * wB * wB + mC * wC * wC + mD * wD * wD;

		// Compute normal impulse.
		float32 impulse = K > 0.0f ? -C / K : 0.0f;

		b3Vec3 PA = impulse * JA;
		b3Vec3 PB = impulse * JB;
		b3Vec3 PC = impulse * JC;
		b3Vec3 PD = impulse * JD;

		xA += mA * PA;
		xB += mB * PB;
		xC += mC * PC;
		xD += mD * PD;

		x[indexA] = xA;
		x[indexB] = xB;
		x[indexC] = xC;
		x[indexD] = xD;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}