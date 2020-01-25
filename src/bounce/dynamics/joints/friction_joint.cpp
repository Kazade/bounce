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

#include <bounce/dynamics/joints/friction_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3FrictionJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
}

b3FrictionJoint::b3FrictionJoint(const b3FrictionJointDef* def)
{
	m_type = b3JointType::e_frictionJoint;
	
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	
	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();

	m_maxForce = def->maxForce;
	m_maxTorque = def->maxTorque;
}

void b3FrictionJoint::InitializeConstraints(const b3SolverData* data)
{
	b3Body* m_bodyA = GetBodyA();
	b3Body* m_bodyB = GetBodyB();

	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_iA = data->invInertias[m_indexA];
	m_iB = data->invInertias[m_indexB];

	b3Vec3 localCenterA = m_bodyA->m_sweep.localCenter;
	b3Vec3 localCenterB = m_bodyB->m_sweep.localCenter;
	
	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	// Compute effective mass for the block solver
	m_rA = b3Mul(qA, m_localAnchorA - localCenterA);
	m_rB = b3Mul(qB, m_localAnchorB - localCenterB);

	b3Mat33 RA = b3Skew(m_rA);
	b3Mat33 RAT = b3Transpose(RA);
	b3Mat33 RB = b3Skew(m_rB);
	b3Mat33 RBT = b3Transpose(RB);
	b3Mat33 M = b3Diagonal(m_mA + m_mB);

	m_linearMass = M + RA * m_iA * RAT + RB * m_iB * RBT;
	m_angularMass = m_iA + m_iB;
}

void b3FrictionJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Vec3 P = m_linearImpulse;

	vA -= m_mA * P;
	wA -= m_iA * (b3Cross(m_rA, P) + m_angularImpulse);

	vB += m_mB * P;
	wB += m_iB * (b3Cross(m_rB, P) + m_angularImpulse);

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3FrictionJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	scalar h = data->dt;

	// Solve angular friction.
	{
		b3Vec3 Cdot = wB - wA;
		b3Vec3 impulse = m_angularMass.Solve(-Cdot);

		b3Vec3 oldImpulse = m_angularImpulse;
		m_angularImpulse += impulse;

		scalar maxImpulse = h * m_maxTorque;
		if (b3LengthSquared(m_angularImpulse) > maxImpulse * maxImpulse)
		{
			m_angularImpulse.Normalize();
			m_angularImpulse *= maxImpulse;
		}

		impulse = m_angularImpulse - oldImpulse;

		wA -= m_iA * impulse;
		wB += m_iB * impulse;
	}

	// Solve linear friction
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_linearMass.Solve(-Cdot);
		b3Vec3 oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		scalar maxImpulse = h * m_maxForce;
		if (b3LengthSquared(m_linearImpulse) > maxImpulse * maxImpulse)
		{
			m_linearImpulse.Normalize();
			m_linearImpulse *= maxImpulse;
		}

		impulse = m_linearImpulse - oldImpulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}
	
	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3FrictionJoint::SolvePositionConstraints(const b3SolverData* data)
{
	B3_NOT_USED(data);
	return true;
}

b3Vec3 b3FrictionJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3FrictionJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

void b3FrictionJoint::SetMaxForce(scalar force)
{
	B3_ASSERT(b3IsValid(force) && force >= scalar(0));
	m_maxForce = force;
}

scalar b3FrictionJoint::GetMaxForce() const
{
	return m_maxForce;
}

void b3FrictionJoint::SetMaxTorque(scalar torque)
{
	B3_ASSERT(b3IsValid(torque) && torque >= scalar(0));
	m_maxTorque = torque;
}

scalar b3FrictionJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void b3FrictionJoint::Draw() const
{
	b3Vec3 a = GetAnchorA();
	b3Draw_draw->DrawPoint(a, scalar(4), b3Color_red);

	b3Vec3 b = GetAnchorB();
	b3Draw_draw->DrawPoint(b, scalar(4), b3Color_green);

	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}