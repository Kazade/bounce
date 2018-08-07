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

#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3SphereJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
}

b3SphereJoint::b3SphereJoint(const b3SphereJointDef* def)
{
	m_type = b3JointType::e_sphereJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_impulse.SetZero();
}

void b3SphereJoint::InitializeConstraints(const b3SolverData* data)
{
	b3Body* m_bodyA = GetBodyA();
	b3Body* m_bodyB = GetBodyB();

	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_localInvIA = m_bodyA->m_invI;
	m_localInvIB = m_bodyB->m_invI;
	m_iA = data->invInertias[m_indexA];
	m_iB = data->invInertias[m_indexB];

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	// Compute effective mass for the block solver
	m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

	b3Mat33 RA = b3Skew(m_rA);
	b3Mat33 RAT = b3Transpose(RA);
	b3Mat33 RB = b3Skew(m_rB);
	b3Mat33 RBT = b3Transpose(RB);
	b3Mat33 M = b3Diagonal(m_mA + m_mB);

	m_mass = M + RA * m_iA * RAT + RB * m_iB * RBT;
}

void b3SphereJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	vA -= m_mA * m_impulse;
	wA -= m_iA * b3Cross(m_rA, m_impulse);

	vB += m_mB * m_impulse;
	wB += m_iB * b3Cross(m_rB, m_impulse);
	
	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3SphereJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
	b3Vec3 impulse = m_mass.Solve(-Cdot);
	
	m_impulse += impulse;

	vA -= m_mA * impulse;
	wA -= m_iA * b3Cross(m_rA, impulse);

	vB += m_mB * impulse;
	wB += m_iB * b3Cross(m_rB, impulse);
	
	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3SphereJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	// Compute effective mass
	b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

	b3Mat33 M = b3Diagonal(m_mA + m_mB);
	b3Mat33 RA = b3Skew(rA);
	b3Mat33 RAT = b3Transpose(RA);
	b3Mat33 RB = b3Skew(rB);
	b3Mat33 RBT = b3Transpose(RB);

	b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;
	
	b3Vec3 C = xB + rB - xA - rA;
	b3Vec3 impulse = mass.Solve(-C);

	xA -= m_mA * impulse;
	qA -= b3Derivative(qA, b3Mul(iA, b3Cross(rA, impulse)));
	qA.Normalize();
	iA = b3RotateToFrame(m_localInvIA, qA);

	xB += m_mB * impulse;
	qB += b3Derivative(qB, b3Mul(iB, b3Cross(rB, impulse)));
	qB.Normalize();
	iB = b3RotateToFrame(m_localInvIB, qB);

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->invInertias[m_indexA] = iA;

	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexB] = iB;

	return b3Length(C) <= B3_LINEAR_SLOP;
}

b3Vec3 b3SphereJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3SphereJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

void b3SphereJoint::Draw() const
{
	b3Vec3 a = GetAnchorA();
	b3Draw_draw->DrawPoint(a, 4.0f, b3Color_red);
	
	b3Vec3 b = GetAnchorB();
	b3Draw_draw->DrawPoint(b, 4.0f, b3Color_green);
	
	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}
