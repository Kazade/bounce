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

#include <bounce/dynamics/joints/motor_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3MotorJointDef::Initialize(b3Body* bA, b3Body* bB)
{
	bodyA = bA;
	bodyB = bB;

	b3Vec3 xB = bodyB->GetPosition();
	linearOffset = bodyA->GetLocalPoint(xB);

	b3Quat qB = bodyB->GetOrientation();
	angularOffset = bodyA->GetLocalFrame(qB);
}

b3MotorJoint::b3MotorJoint(const b3MotorJointDef* def)
{
	m_type = e_motorJoint;
	m_linearOffset = def->linearOffset;
	m_angularOffset = def->angularOffset;
	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();

	m_maxForce = def->maxForce;
	m_maxTorque = def->maxTorque;
	m_correctionFactor = def->correctionFactor;
}

void b3MotorJoint::InitializeConstraints(const b3SolverData* data)
{
	scalar inv_h = data->invdt;

	b3Body* m_bodyA = GetBodyA();
	b3Body* m_bodyB = GetBodyB();

	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_iA = data->invInertias[m_indexA];
	m_iB = data->invInertias[m_indexB];
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;

	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;

	{
		// Compute effective mass for the block solver
		m_rA = b3Mul(qA, m_linearOffset - m_localCenterA);
		m_rB = b3Mul(qB, -m_localCenterB);

		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Diagonal(m_mA + m_mB);

		m_linearMass = M + RA * m_iA * RAT + RB * m_iB * RBT;
		m_linearError = xB + m_rB - xA - m_rA;
	}

	{
		b3Quat q1 = b3Conjugate(qA) * qB;
		b3Quat q2 = m_angularOffset;

		if (b3Dot(q1, q2) < scalar(0))
		{
			q1 = -q1;
		}

		// Apply finite difference
		
		// Angular velocity that will rotate q1 to q2 over h
		b3Quat qw = inv_h * scalar(2) * (q2 - q1) * b3Conjugate(q1);

		// Convert the relative velocity to world's frame
		m_angularVelocity = b3Mul(qA, qw.v);

		m_angularVelocity *= m_correctionFactor;

		m_angularMass = m_iA + m_iB;
	}
}

void b3MotorJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	{
		vA -= m_mA * m_linearImpulse;
		wA -= m_iA * b3Cross(m_rA, m_linearImpulse);

		vB += m_mB * m_linearImpulse;
		wB += m_iB * b3Cross(m_rB, m_linearImpulse);
	}

	{
		wA -= m_iA * m_angularImpulse;
		wB += m_iB * m_angularImpulse;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3MotorJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	scalar h = data->dt;
	scalar inv_h = data->invdt;

	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	// Solve linear friction
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;
		
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

	// Solve angular friction
	{
		b3Vec3 Cdot = (wB - wA) - m_angularVelocity;

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

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3MotorJoint::SolvePositionConstraints(const b3SolverData* data)
{
	B3_NOT_USED(data);
	return true;
}

b3Vec3 b3MotorJoint::GetAnchorA() const
{
	return GetBodyA()->GetPosition();
}

b3Vec3 b3MotorJoint::GetAnchorB() const
{
	return GetBodyB()->GetPosition();
}

void b3MotorJoint::SetLinearOffset(const b3Vec3& linearOffset)
{
	if (linearOffset.x != m_linearOffset.x || 
		linearOffset.y != m_linearOffset.y || 
		linearOffset.z != m_linearOffset.z)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_linearOffset = linearOffset;
	}
}

const b3Vec3& b3MotorJoint::GetLinearOffset() const
{
	return m_linearOffset;
}

void b3MotorJoint::SetAngularOffset(const b3Quat& angularOffset)
{
	if (angularOffset.v.x != m_angularOffset.v.x ||
		angularOffset.v.y != m_angularOffset.v.y ||
		angularOffset.v.z != m_angularOffset.v.z ||
		angularOffset.s != m_angularOffset.s)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_angularOffset = angularOffset;
	}
}

const b3Quat& b3MotorJoint::GetAngularOffset() const
{
	return m_angularOffset;
}

void b3MotorJoint::SetMaxTorque(scalar torque)
{
	B3_ASSERT(b3IsValid(torque) && torque >= scalar(0));
	m_maxTorque = torque;
}

scalar b3MotorJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

void b3MotorJoint::SetMaxForce(scalar force)
{
	B3_ASSERT(b3IsValid(force) && force >= scalar(0));
	m_maxForce = force;
}

scalar b3MotorJoint::GetMaxForce() const
{
	return m_maxForce;
}

void b3MotorJoint::SetCorrectionFactor(scalar factor)
{
	B3_ASSERT(b3IsValid(factor) && factor >= scalar(0) && factor <= scalar(1));
	m_correctionFactor = factor;
}

scalar b3MotorJoint::GetCorrectionFactor() const
{
	return m_correctionFactor;
}

void b3MotorJoint::Draw() const
{
	b3Vec3 a = GetAnchorA();
	b3Draw_draw->DrawPoint(a, scalar(4), b3Color_red);

	b3Vec3 b = GetAnchorB();
	b3Draw_draw->DrawPoint(b, scalar(4), b3Color_green);

	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}
