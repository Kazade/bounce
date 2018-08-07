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

#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

// C = dot(u2, u1) - cos(angle / 2) > 0
// Cdot = dot(u2, omega1 x u1) + dot(u1, omega2 x u2)
// Cycle:
// dot(u1 x u2, omega1) + dot(u2 x u1, omega2) = 
// dot(-u2 x u1, omega1) + dot(u2 x u1, omega2)
// n = u2 x u1
// J = [0 -n 0 n]

// Stable C: 
// C =  angle / 2 - atan2( norm(u2 x u1), dot(u2, u1) ) > 0

void b3ConeJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor, float32 angle)
{
	bodyA = bA;
	bodyB = bB;

	b3Transform xf;
	xf.rotation.y = axis;
	xf.rotation.z = b3Perp(axis);
	xf.rotation.x = b3Cross(xf.rotation.z, xf.rotation.y);
	xf.position = anchor;

	localFrameA = bodyA->GetLocalFrame(xf);
	localFrameB = bodyB->GetLocalFrame(xf);
	coneAngle = angle;
}

b3ConeJoint::b3ConeJoint(const b3ConeJointDef* def)
{
	m_type = e_coneJoint;
	m_localFrameA = def->localFrameA;
	m_localFrameB = def->localFrameB;
	m_enableLimit = def->enableLimit;
	m_coneAngle = def->coneAngle;
	m_limitState = e_inactiveLimit;
	m_limitImpulse = 0.0f;
	m_limitAxis.SetZero();
	m_impulse.SetZero();
}

void b3ConeJoint::InitializeConstraints(const b3SolverData* data)
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

	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;

	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;

	b3Transform xfA = m_bodyA->GetWorldFrame(m_localFrameA);
	b3Transform xfB = m_bodyB->GetWorldFrame(m_localFrameB);

	// Add point-to-point constraint.
	{
		m_rA = b3Mul(qA, m_localFrameA.position - m_localCenterA);
		m_rB = b3Mul(qB, m_localFrameB.position - m_localCenterB);

		// Compute effective mass matrix.
		b3Mat33 M = b3Diagonal(m_mA + m_mB);
		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		m_mass = M + RA * m_iA * RAT + RB * m_iB * RBT;
	}

	// Add limit constraint.
	if (m_enableLimit)
	{
		b3Vec3 u1 = xfA.rotation.y;
		b3Vec3 u2 = xfB.rotation.y;

		m_limitAxis = b3Cross(u2, u1);

		float32 mass = b3Dot((m_iA + m_iB) * m_limitAxis, m_limitAxis);
		m_limitMass = mass > 0.0f ? 1.0f / mass : 0.0f;

		// C = cone / 2 - angle >= 0
		float32 cosine = b3Dot(u2, u1);
		float32 sine = b3Length(m_limitAxis);
		float32 angle = atan2(sine, cosine);
		if (0.5f * m_coneAngle < angle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_limitImpulse = 0.0f;
			}
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_limitImpulse = 0.0f;
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}
}

void b3ConeJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	{
		vA -= m_mA * m_impulse;
		wA -= m_iA * b3Cross(m_rA, m_impulse);

		vB += m_mB * m_impulse;
		wB += m_iB * b3Cross(m_rB, m_impulse);
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b3Vec3 P = m_limitImpulse * m_limitAxis;
		wA -= m_iA * P;
		wB += m_iB * P;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3ConeJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	// Solve point-to-point constraint.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 P = m_mass.Solve(-Cdot);

		m_impulse += P;

		vA -= m_mA * P;
		wA -= m_iA * b3Cross(m_rA, P);

		vB += m_mB * P;
		wB += m_iB * b3Cross(m_rB, P);
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 Cdot = b3Dot(m_limitAxis, wB - wA);
		float32 impulse = -m_limitMass * Cdot;
		float32 oldImpulse = m_limitImpulse;
		m_limitImpulse = b3Max(m_limitImpulse + impulse, 0.0f);
		impulse = m_limitImpulse - oldImpulse;

		b3Vec3 P = impulse * m_limitAxis;

		wA -= m_iA * P;
		wB += m_iB * P;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3ConeJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	float32 mA = m_mA;
	float32 mB = m_mB;
	
	// Solve point-to-point constraint.
	float32 linearError = 0.0f;
	{
		b3Vec3 rA = b3Mul(qA, m_localFrameA.position - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localFrameB.position - m_localCenterB);

		b3Vec3 C = xB + rB - xA - rA;

		linearError = b3Length(C);

		b3Mat33 M = b3Diagonal(mA + mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;

		b3Vec3 P = mass.Solve(-C);

		xA -= mA * P;
		qA -= b3Derivative(qA, iA * b3Cross(rA, P));
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * P;
		qB += b3Derivative(qB, iB * b3Cross(rB, P));
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	// Solve limit constraint.
	float32 limitError = 0.0f;
	if (m_enableLimit)
	{
		// Compute fresh Jacobian
		b3Vec3 u1 = b3Mul(qA, m_localFrameA.rotation.y);
		b3Vec3 u2 = b3Mul(qB, m_localFrameB.rotation.y);
		b3Vec3 limitAxis = b3Cross(u2, u1);

		// Compute fresh effective mass.
		float32 mass = b3Dot((iA + iB) * limitAxis, limitAxis);
		float32 limitMass = mass > 0.0f ? 1.0f / mass : 0.0f;

		// Compute joint angle.
		float32 cosine = b3Dot(u2, u1);
		float32 sine = b3Length(limitAxis);
		float32 angle = atan2(sine, cosine);

		float32 limitImpulse = 0.0f;

		if (0.5f * m_coneAngle < angle)
		{
			float32 C = 0.5f * m_coneAngle - angle;
			limitError = -C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C + B3_ANGULAR_SLOP, -B3_MAX_ANGULAR_CORRECTION, 0.0f);
			limitImpulse = -C * limitMass;
		}

		b3Vec3 P = limitImpulse * limitAxis;

		qA -= b3Derivative(qA, iA * P);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * P);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && limitError <= B3_ANGULAR_SLOP;
}

b3Transform b3ConeJoint::GetFrameA() const
{
	return GetBodyA()->GetWorldFrame(m_localFrameA);
}

b3Transform b3ConeJoint::GetFrameB() const
{
	return GetBodyB()->GetWorldFrame(m_localFrameB);
}

const b3Transform& b3ConeJoint::GetLocalFrameA() const
{
	return m_localFrameA;
}

const b3Transform& b3ConeJoint::GetLocalFrameB() const
{
	return m_localFrameB;
}

bool b3ConeJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b3ConeJoint::SetEnableLimit(bool bit)
{
	if (bit != m_enableLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = 0.0f;
		m_limitState = e_inactiveLimit;
		m_enableLimit = bit;
	}
}

float32 b3ConeJoint::GetConeAngle() const
{
	return m_coneAngle;
}

void b3ConeJoint::SetConeAngle(float32 angle)
{
	if (angle != m_coneAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = 0.0f;
		m_coneAngle = angle;
	}
}

void b3ConeJoint::Draw() const
{
	b3Transform xfA = GetFrameA();
	b3Draw_draw->DrawTransform(xfA);
	b3Transform xfB = GetFrameB();
	b3Draw_draw->DrawTransform(xfB);
}