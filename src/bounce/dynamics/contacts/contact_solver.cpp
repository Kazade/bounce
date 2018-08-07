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

#include <bounce/dynamics/contacts/contact_solver.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/math/mat.h>

// This solver implements PGS for solving velocity constraints and 
// NGS for solving position constraints.

b3ContactSolver::b3ContactSolver(const b3ContactSolverDef* def)
{
	m_allocator = def->allocator;
	m_count = def->count;
	m_positions = def->positions;
	m_velocities = def->velocities;
	m_inertias = def->invInertias;
	m_contacts = def->contacts;
	m_positionConstraints = (b3ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(b3ContactPositionConstraint));
	m_velocityConstraints = (b3ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(b3ContactVelocityConstraint));
	m_dt = def->dt;
	m_invDt = m_dt != 0.0f ? 1.0f / m_dt : 0.0f;
}

b3ContactSolver::~b3ContactSolver()
{
	// Reverse free.
	for (u32 index1 = m_count; index1 > 0; --index1)
	{
		u32 i1 = index1 - 1;
		b3ContactPositionConstraint* pc = m_positionConstraints + i1;
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i1;
		
		for (u32 index2 = pc->manifoldCount; index2 > 0; --index2)
		{
			u32 i2 = index2 - 1;
			b3PositionConstraintManifold* pcm = pc->manifolds + i2;
			b3VelocityConstraintManifold* vcm = vc->manifolds + i2;
			
			m_allocator->Free(vcm->points);
			m_allocator->Free(pcm->points);
		}

		m_allocator->Free(vc->manifolds);
		m_allocator->Free(pc->manifolds);
	}

	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_positionConstraints);
}

void b3ContactSolver::InitializeConstraints()
{
	for (u32 i = 0; i < m_count; ++i)
	{
		b3Contact* c = m_contacts[i];

		b3Shape* shapeA = c->GetShapeA();
		b3Shape* shapeB = c->GetShapeB();

		b3Body* bodyA = shapeA->GetBody();
		b3Body* bodyB = shapeB->GetBody();

		u32 manifoldCount = c->m_manifoldCount;
		b3Manifold* manifolds = c->m_manifolds;

		b3ContactPositionConstraint* pc = m_positionConstraints + i;
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		pc->indexA = bodyA->m_islandID;
		pc->invMassA = bodyA->m_invMass;
		pc->localInvIA = bodyA->m_invI;
		pc->localCenterA = bodyA->m_sweep.localCenter;
		pc->radiusA = shapeA->m_radius;

		pc->indexB = bodyB->m_islandID;
		pc->invMassB = bodyB->m_invMass;
		pc->localInvIB = bodyB->m_invI;
		pc->localCenterB = bodyB->m_sweep.localCenter;
		pc->radiusB = shapeB->m_radius;

		pc->manifoldCount = manifoldCount;
		pc->manifolds = (b3PositionConstraintManifold*)m_allocator->Allocate(manifoldCount * sizeof(b3PositionConstraintManifold));

		vc->indexA = bodyA->m_islandID;
		vc->invMassA = bodyA->m_invMass;
		vc->invIA = m_inertias[vc->indexA];

		vc->indexB = bodyB->m_islandID;
		vc->invMassB = bodyB->m_invMass;
		vc->invIB = m_inertias[vc->indexB];

		vc->friction = b3MixFriction(shapeA->m_friction, shapeB->m_friction);
		vc->restitution = b3MixRestitution(shapeA->m_restitution, shapeB->m_restitution);

		vc->manifoldCount = manifoldCount;
		vc->manifolds = (b3VelocityConstraintManifold*)m_allocator->Allocate(manifoldCount * sizeof(b3VelocityConstraintManifold));

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3Manifold* m = manifolds + j;
			b3PositionConstraintManifold* pcm = pc->manifolds + j;
			b3VelocityConstraintManifold* vcm = vc->manifolds + j;

			pcm->pointCount = m->pointCount;
			pcm->points = (b3PositionConstraintPoint*)m_allocator->Allocate(pcm->pointCount * sizeof(b3PositionConstraintPoint));
			
			vcm->pointCount = m->pointCount;
			vcm->points = (b3VelocityConstraintPoint*)m_allocator->Allocate(vcm->pointCount * sizeof(b3VelocityConstraintPoint));
			
			vcm->tangentImpulse = m->tangentImpulse;
			vcm->motorImpulse = m->motorImpulse;
			
			for (u32 k = 0; k < m->pointCount; ++k)
			{
				b3ManifoldPoint* cp = m->points + k;
				b3PositionConstraintPoint* pcp = pcm->points + k;
				b3VelocityConstraintPoint* vcp = vcm->points + k;

				pcp->localNormalA = cp->localNormal1;
				pcp->localPointA = cp->localPoint1;
				pcp->localPointB = cp->localPoint2;

				vcp->normalImpulse = cp->normalImpulse;
			}
		}
	}

	for (u32 i = 0; i < m_count; ++i)
	{
		b3Contact* c = m_contacts[i];
		u32 manifoldCount = c->m_manifoldCount;

		b3ContactPositionConstraint* pc = m_positionConstraints + i;
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		float32 radiusA = pc->radiusA;
		float32 radiusB = pc->radiusB;

		b3Vec3 localCenterA = pc->localCenterA;
		b3Vec3 localCenterB = pc->localCenterB;

		u32 indexA = vc->indexA;
		float32 mA = vc->invMassA;
		b3Mat33 iA = vc->invIA;

		u32 indexB = vc->indexB;
		float32 mB = vc->invMassB;
		b3Mat33 iB = vc->invIB;

		b3Vec3 xA = m_positions[indexA].x;
		b3Quat qA = m_positions[indexA].q;
		b3Vec3 xB = m_positions[indexB].x;
		b3Quat qB = m_positions[indexB].q;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;

		b3Transform xfA;
		xfA.rotation = b3QuatMat33(qA);
		xfA.position = xA - b3Mul(xfA.rotation, localCenterA);
		
		b3Transform xfB;
		xfB.rotation = b3QuatMat33(qB);
		xfB.position = xB - b3Mul(xfB.rotation, localCenterB);

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3Manifold* m = c->m_manifolds + j;
			b3VelocityConstraintManifold* vcm = vc->manifolds + j;

			b3WorldManifold wm;
			wm.Initialize(m, radiusA, xfA, radiusB, xfB);

			vcm->normal = wm.normal;
			vcm->tangent1 = wm.tangent1;
			vcm->tangent2 = wm.tangent2;

			u32 pointCount = wm.pointCount;

			for (u32 k = 0; k < pointCount; ++k)
			{
				b3WorldManifoldPoint* mp = wm.points + k;

				b3VelocityConstraintPoint* vcp = vcm->points + k;

				b3Vec3 normal = mp->normal;
				b3Vec3 point = mp->point;
				
				b3Vec3 rA = point - xA;
				b3Vec3 rB = point - xB;

				vcp->rA = rA;
				vcp->rB = rB;

				// Add normal constraint.
				{
					vcp->normal = normal;

					// Compute effective mass.
					b3Vec3 rnA = b3Cross(rA, normal);
					b3Vec3 rnB = b3Cross(rB, normal);
					float32 K = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);
					vcp->normalMass = K > 0.0f ? 1.0f / K : 0.0f;

					// Add restitution to the velocity constraint.
					b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);
					float32 vn = b3Dot(normal, dv);
					vcp->velocityBias = 0.0f;
					if (vn < -B3_VELOCITY_THRESHOLD)
					{
						vcp->velocityBias = -vc->restitution * vn;
					}
				}
			}

			B3_ASSERT(pointCount > 0);
			
			// Add friction constraints.	
			if(pointCount > 0)
			{
				b3Vec3 rA = wm.center - xA;
				b3Vec3 rB = wm.center - xB;

				vcm->rA = rA;
				vcm->rB = rB;

				{
					b3Vec3 t1 = vcm->tangent1;
					b3Vec3 t2 = vcm->tangent2;
					
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

					vcm->tangentMass = b3Inverse(K);
				}

				// Add twist constraint.
				{
					float32 mass = b3Dot(vcm->normal, (iA + iB) * vcm->normal);
					vcm->motorMass = mass > 0.0f ? 1.0f / mass : 0.0f;
				}
			}
		}
	}
}

void b3ContactSolver::WarmStart()
{
	for (u32 i = 0; i < m_count; ++i)
	{
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		u32 indexA = vc->indexA;
		float32 mA = vc->invMassA;
		b3Mat33 iA = vc->invIA;

		u32 indexB = vc->indexB;
		float32 mB = vc->invMassB;
		b3Mat33 iB = vc->invIB;

		u32 manifoldCount = vc->manifoldCount;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3VelocityConstraintManifold* vcm = vc->manifolds + j;
			u32 pointCount = vcm->pointCount;

			for (u32 k = 0; k < pointCount; ++k)
			{
				b3VelocityConstraintPoint* vcp = vcm->points + k;

				b3Vec3 P = vcp->normalImpulse * vcp->normal;
				
				vA -= mA * P;
				wA -= iA * b3Cross(vcp->rA, P);

				vB += mB * P;
				wB += iB * b3Cross(vcp->rB, P);
			}

			if (pointCount > 0)
			{
				b3Vec3 P1 = vcm->tangentImpulse.x * vcm->tangent1;
				b3Vec3 P2 = vcm->tangentImpulse.y * vcm->tangent2;
				b3Vec3 P3 = vcm->motorImpulse * vcm->normal;
				
				vA -= mA * (P1 + P2);
				wA -= iA * (b3Cross(vcm->rA, P1 + P2) + P3);

				vB += mB * (P1 + P2);
				wB += iB * (b3Cross(vcm->rB, P1 + P2) + P3);
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b3ContactSolver::SolveVelocityConstraints()
{
	for (u32 i = 0; i < m_count; ++i)
	{
		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;
		u32 manifoldCount = vc->manifoldCount;

		u32 indexA = vc->indexA;
		float32 mA = vc->invMassA;
		b3Mat33 iA = vc->invIA;

		u32 indexB = vc->indexB;
		float32 mB = vc->invMassB;
		b3Mat33 iB = vc->invIB;

		b3Vec3 vA = m_velocities[indexA].v;
		b3Vec3 wA = m_velocities[indexA].w;
		b3Vec3 vB = m_velocities[indexB].v;
		b3Vec3 wB = m_velocities[indexB].w;

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3VelocityConstraintManifold* vcm = vc->manifolds + j;
			u32 pointCount = vcm->pointCount;

			float32 normalImpulse = 0.0f;
			for (u32 k = 0; k < pointCount; ++k)
			{
				b3VelocityConstraintPoint* vcp = vcm->points + k;
				B3_ASSERT(vcp->normalImpulse >= 0.0f);

				// Solve normal constraints.
				{
					b3Vec3 dv = vB + b3Cross(wB, vcp->rB) - vA - b3Cross(wA, vcp->rA);
					float32 Cdot = b3Dot(vcp->normal, dv);

					float32 impulse = -vcp->normalMass * (Cdot - vcp->velocityBias);

					float32 oldImpulse = vcp->normalImpulse;
					vcp->normalImpulse = b3Max(vcp->normalImpulse + impulse, 0.0f);
					impulse = vcp->normalImpulse - oldImpulse;

					b3Vec3 P = impulse * vcp->normal;

					vA -= mA * P;
					wA -= iA * b3Cross(vcp->rA, P);

					vB += mB * P;
					wB += iB * b3Cross(vcp->rB, P);

					normalImpulse += vcp->normalImpulse;
				}
			}
			
			if (pointCount > 0)
			{
				// Solve tangent constraints.
				{
					b3Vec3 dv = vB + b3Cross(wB, vcm->rB) - vA - b3Cross(wA, vcm->rA);
					
					b3Vec2 Cdot;
					Cdot.x = b3Dot(dv, vcm->tangent1);
					Cdot.y = b3Dot(dv, vcm->tangent2);

					b3Vec2 impulse = vcm->tangentMass * -Cdot;
					b3Vec2 oldImpulse = vcm->tangentImpulse;
					vcm->tangentImpulse += impulse;
					
					float32 maxImpulse = vc->friction * normalImpulse;
					if (b3Dot(vcm->tangentImpulse, vcm->tangentImpulse) > maxImpulse * maxImpulse)
					{
						vcm->tangentImpulse.Normalize();
						vcm->tangentImpulse *= maxImpulse;
					}
					
					impulse = vcm->tangentImpulse - oldImpulse;

					b3Vec3 P1 = impulse.x * vcm->tangent1;
					b3Vec3 P2 = impulse.y * vcm->tangent2;
					b3Vec3 P = P1 + P2;

					vA -= mA * P;
					wA -= iA * b3Cross(vcm->rA, P);

					vB += mB * P;
					wB += iB * b3Cross(vcm->rB, P);
				}

				// Solve motor constraint.
				{
					float32 Cdot = b3Dot(vcm->normal, wB - wA);
					float32 impulse = -vcm->motorMass * Cdot;
					float32 oldImpulse = vcm->motorImpulse;
					float32 maxImpulse = vc->friction * normalImpulse;
					vcm->motorImpulse = b3Clamp(vcm->motorImpulse + impulse, -maxImpulse, maxImpulse);
					impulse = vcm->motorImpulse - oldImpulse;

					b3Vec3 P = impulse * vcm->normal;

					wA -= iA * P;
					wB += iB * P;
				}
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void b3ContactSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_count; ++i)
	{
		b3Contact* c = m_contacts[i];
		b3Manifold* manifolds = c->m_manifolds;
		u32 manifoldCount = c->m_manifoldCount;

		b3ContactVelocityConstraint* vc = m_velocityConstraints + i;

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3Manifold* m = manifolds + j;
			u32 pointCount = m->pointCount;

			b3VelocityConstraintManifold* vcm = vc->manifolds + j;
			m->tangentImpulse = vcm->tangentImpulse;
			m->motorImpulse = vcm->motorImpulse;

			for (u32 k = 0; k < pointCount; ++k)
			{
				b3ManifoldPoint* cp = m->points + k;
				b3VelocityConstraintPoint* vcp = vcm->points + k;
				cp->normalImpulse = vcp->normalImpulse;
			}
		}
	}
}

struct b3ContactPositionSolverPoint
{
	void Initialize(const b3ContactPositionConstraint* pc, const b3PositionConstraintPoint* pcp, const b3Transform& xfA, const b3Transform& xfB)
	{
		normal = b3Mul(xfA.rotation, pcp->localNormalA);
		b3Vec3 c1 = b3Mul(xfA, pcp->localPointA);
		b3Vec3 c2 = b3Mul(xfB, pcp->localPointB);
		point = c2;
		separation = b3Dot(c2 - c1, normal) - pc->radiusA - pc->radiusB;
	}

	b3Vec3 normal;
	b3Vec3 point;
	float32 separation;
};

bool b3ContactSolver::SolvePositionConstraints()
{
	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_count; ++i)
	{
		b3ContactPositionConstraint* pc = m_positionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		b3Vec3 localCenterA = pc->localCenterA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;
		b3Vec3 localCenterB = pc->localCenterB;

		b3Vec3 cA = m_positions[indexA].x;
		b3Quat qA = m_positions[indexA].q;
		b3Mat33 iA = m_inertias[indexA];

		b3Vec3 cB = m_positions[indexB].x;
		b3Quat qB = m_positions[indexB].q;
		b3Mat33 iB = m_inertias[indexB];

		u32 manifoldCount = pc->manifoldCount;

		for (u32 j = 0; j < manifoldCount; ++j)
		{
			b3PositionConstraintManifold* pcm = pc->manifolds + j;
			u32 pointCount = pcm->pointCount;

			// Solve normal constraints
			for (u32 k = 0; k < pointCount; ++k)
			{
				b3PositionConstraintPoint* pcp = pcm->points + k;

				b3Transform xfA;
				xfA.rotation = b3QuatMat33(qA);
				xfA.position = cA - b3Mul(xfA.rotation, localCenterA);

				b3Transform xfB;
				xfB.rotation = b3QuatMat33(qB);
				xfB.position = cB - b3Mul(xfB.rotation, localCenterB);

				b3ContactPositionSolverPoint cpcp;
				cpcp.Initialize(pc, pcp, xfA, xfB);

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
				iA = b3RotateToFrame(pc->localInvIA, qA);

				cB += mB * P;
				qB += b3Derivative(qB, iB * b3Cross(rB, P));
				qB.Normalize();
				iB = b3RotateToFrame(pc->localInvIB, qB);
			}
		}

		m_positions[indexA].x = cA;
		m_positions[indexA].q = qA;
		m_inertias[indexA] = iA;

		m_positions[indexB].x = cB;
		m_positions[indexB].q = qB;
		m_inertias[indexB] = iB;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}
