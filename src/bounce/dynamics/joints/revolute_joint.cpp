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

#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3RevoluteJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor,
	scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);

	bodyA = bA;
	bodyB = bB;

	b3Mat33 rotation;
	rotation.z = axis;
	b3ComputeBasis(rotation.z, rotation.x, rotation.y);

	b3Quat q = b3Mat33Quat(rotation);

	localAnchorA = bodyA->GetLocalPoint(anchor);
	localRotationA = bodyA->GetLocalFrame(q);

	localAnchorB = bodyB->GetLocalPoint(anchor);
	localRotationB = bodyB->GetLocalFrame(q);

	referenceRotation.SetIdentity();

	lowerAngle = lower;
	upperAngle = upper;
}

b3RevoluteJoint::b3RevoluteJoint(const b3RevoluteJointDef* def)
{
	m_type = e_revoluteJoint;
	m_referenceRotation = def->referenceRotation;
	m_localAnchorA = def->localAnchorA;
	m_localRotationA = def->localRotationA;
	m_localAnchorB = def->localAnchorB;
	m_localRotationB = def->localRotationB;

	m_enableMotor = def->enableMotor;
	m_motorSpeed = def->motorSpeed;
	m_maxMotorTorque = def->maxMotorTorque;

	m_enableLimit = def->enableLimit;
	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	B3_ASSERT(m_lowerAngle <= m_upperAngle);
	
	m_motorImpulse = scalar(0);

	m_limitState = e_inactiveLimit;
	m_limitImpulse = scalar(0);
	
	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();
}

void b3RevoluteJoint::InitializeConstraints(const b3SolverData* data)
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

	scalar mA = m_mA;
	b3Mat33 iA = m_iA;
	scalar mB = m_mB;
	b3Mat33 iB = m_iB;

	b3Quat fA = qA * m_localRotationA;
	b3Quat fB = qB * m_localRotationB;

	b3Mat33 RfA = b3QuatMat33(fA);
	b3Mat33 RfB = b3QuatMat33(fB);

	b3Vec3 u1 = RfA.x;
	b3Vec3 v1 = RfA.y;
	b3Vec3 w1 = RfA.z;

	b3Vec3 u2 = RfB.x;
	b3Vec3 v2 = RfB.y;
	b3Vec3 w2 = RfB.z;
	
	// Motor constraint.
	if (m_enableMotor || m_enableLimit)
	{
		m_motorAxis = w1;
		m_motorAxis.Normalize();

		scalar K = b3Dot((iA + iB) * m_motorAxis, m_motorAxis);
		
		m_motorMass = K > scalar(0) ? scalar(1) / K : scalar(0);
	}
		
	// Limit constraint.
	if (m_enableLimit)
	{
		// Joint rotation
		b3Quat q = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;
		
		// Joint angle
		scalar angle = scalar(2) * atan2(q.v.z, q.s);

		if (b3Abs(m_upperAngle - m_lowerAngle) < scalar(2) * B3_ANGULAR_SLOP)
		{
			if (m_limitState != e_equalLimits)
			{
				m_limitState = e_equalLimits;
				m_limitImpulse = scalar(0);
			}
		}
		else if (angle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_limitImpulse = scalar(0);
			}
		}
		else if (angle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_limitState = e_atUpperLimit;
				m_limitImpulse = scalar(0);
			}
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_limitImpulse = scalar(0);
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}

	// Linear constraints.
	{
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Diagonal(mA + mB);

		m_linearMass = M + RA * iA * RAT + RB * iB * RBT;
	}

	// Angular constraints.
	{
		b3Vec3 s2 = b3Cross(u2, w1);
		b3Vec3 s4 = b3Cross(v2, w1);

		scalar k11 = b3Dot(s2, (iA + iB) * s2);
		scalar k12 = b3Dot(s2, iA * s4) + b3Dot(s2, iB * s4);
		scalar k22 = b3Dot(s4, (iA + iB) * s4);

		b3Mat22 K;
		K.x.x = k11;
		K.x.y = k12;
		K.y.x = k12;
		K.y.y = k22;

		m_a1 = s2;
		m_a2 = s4;
		m_angularMass = b3Inverse(K);
	}
}

void b3RevoluteJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		b3Vec3 P = m_motorAxis * m_motorImpulse;

		wA -= m_iA * P;
		wB += m_iB * P;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b3Vec3 P = m_motorAxis * m_limitImpulse;

		wA -= m_iA * P;
		wB += m_iB * P;
	}

	{
		vA -= m_mA * m_linearImpulse;
		wA -= m_iA * b3Cross(m_rA, m_linearImpulse);

		vB += m_mB * m_linearImpulse;
		wB += m_iB * b3Cross(m_rB, m_linearImpulse);
	}

	{
		b3Vec3 L = m_angularImpulse.x * m_a1 + m_angularImpulse.y * m_a2;

		wA -= m_iA * L;
		wB += m_iB * L;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3RevoluteJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Mat33 iA = m_iA;
	b3Mat33 iB = m_iB;

	// Solve motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		scalar dw = b3Dot(wB - wA, m_motorAxis);
		scalar Cdot = dw - m_motorSpeed;
		scalar impulse = -m_motorMass * Cdot;
		scalar oldImpulse = m_motorImpulse;
		scalar maxImpulse = data->dt * m_maxMotorTorque;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P = m_motorAxis * impulse;

		wA -= iA * P;
		wB += iB * P;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		scalar Cdot = b3Dot(wB - wA, m_motorAxis);
		scalar impulse = -m_motorMass * Cdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			scalar oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Max(m_limitImpulse + impulse, scalar(0));
			impulse = m_limitImpulse - oldImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			scalar oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Min(m_limitImpulse + impulse, scalar(0));
			impulse = m_limitImpulse - oldImpulse;
		}

		b3Vec3 P = impulse * m_motorAxis;

		wA -= iA * P;
		wB += iB * P;
	}

	// Solve linear constraints.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_linearMass.Solve(-Cdot);

		m_linearImpulse += impulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}

	// Solve angular constraints.
	{
		b3Vec3 dw = wB - wA;

		b3Vec2 Cdot;
		Cdot.x = b3Dot(dw, m_a1);
		Cdot.y = b3Dot(dw, m_a2);

		b3Vec2 impulse = m_angularMass * -Cdot;

		m_angularImpulse += impulse;

		b3Vec3 L = impulse.x * m_a1 + impulse.y * m_a2;

		wA -= m_iA * L;
		wB += m_iB * L;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3RevoluteJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	scalar mA = m_mA;
	scalar mB = m_mB;

	// Solve angular constraints.
	scalar angularError = scalar(0);

	{
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;

		b3Quat q1 = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;
		
		b3Quat q2;
		q2.v.x = scalar(0);
		q2.v.y = scalar(0);
		q2.v.z = q1.v.z;
		q2.s = q1.s;

		// Solve limit constraint
		if (m_enableLimit)
		{
			// Joint angle
			scalar angle = scalar(2) * atan2(q2.v.z, q2.s);

			scalar C = scalar(0);
			if (b3Abs(m_upperAngle - m_lowerAngle) < scalar(2) * B3_ANGULAR_SLOP)
			{
				C = angle - m_lowerAngle;

				// Prevent large corrections
				C = b3Clamp(C, -B3_MAX_ANGULAR_CORRECTION, B3_MAX_ANGULAR_CORRECTION);
			}
			else if (angle <= m_lowerAngle)
			{
				C = angle - m_lowerAngle;

				// Allow some slop and prevent large corrections
				C = b3Clamp(C + B3_ANGULAR_SLOP, -B3_MAX_ANGULAR_CORRECTION, scalar(0));
			}
			else if (angle >= m_upperAngle)
			{
				C = angle - m_upperAngle;

				// Allow some slop and prevent large corrections
				C = b3Clamp(C - B3_ANGULAR_SLOP, scalar(0), B3_MAX_ANGULAR_CORRECTION);
			}

			if (C != scalar(0))
			{
				scalar theta = scalar(0.5) * (angle - C);

				q2.v.z = sin(theta);
				q2.s = cos(theta);
			}
		}

		if (b3Dot(q1, q2) < scalar(0))
		{
			q1 = -q1;
		}

		// d * q1 = q2
		// d = q2 * q1^-1
		b3Quat d = q2 * b3Conjugate(q1);

		// Exact local errors
		b3Vec3 v;
		v.x = scalar(2) * atan2(d.v.x, d.s);
		v.y = scalar(2) * atan2(d.v.y, d.s);
		v.z = scalar(2) * atan2(d.v.z, d.s);

		angularError += b3Length(v);

		// Convert the local angular error to world's frame
		// Negate the local error.
		b3Vec3 C = b3Mul(fA, -v);

		b3Mat33 mass = iA + iB;

		b3Vec3 impulse = mass.Solve(-C);

		qA -= b3Derivative(qA, iA * impulse);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * impulse);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	// Solve linear constraints.
	scalar linearError = scalar(0);

	{
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Vec3 C = xB + rB - xA - rA;

		linearError += b3Length(C);

		// Compute effective mass
		b3Mat33 M = b3Diagonal(mA + mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);

		b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;

		b3Vec3 impulse = mass.Solve(-C);

		xA -= mA * impulse;
		qA -= b3Derivative(qA, iA * b3Cross(rA, impulse));
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * impulse;
		qB += b3Derivative(qB, iB * b3Cross(rB, impulse));
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && angularError <= B3_ANGULAR_SLOP;
}

b3Transform b3RevoluteJoint::GetFrameA() const
{
	b3Transform xf(m_localAnchorA, m_localRotationA);
	return GetBodyA()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetFrameB() const
{
	b3Transform xf(m_localAnchorB, m_localRotationB);
	return GetBodyB()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetLocalFrameA() const
{
	return b3Transform(m_localAnchorA, m_localRotationA);
}

b3Transform b3RevoluteJoint::GetLocalFrameB() const
{
	return b3Transform(m_localAnchorB, m_localRotationB);
}

bool b3RevoluteJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b3RevoluteJoint::SetEnableLimit(bool bit)
{
	if (bit != m_enableLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = scalar(0);
		m_limitState = e_inactiveLimit;
		m_enableLimit = bit;
	}
}

scalar b3RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

scalar b3RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void b3RevoluteJoint::SetLimits(scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);
	if (lower != m_lowerAngle || upper != m_upperAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = scalar(0);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

bool b3RevoluteJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b3RevoluteJoint::SetEnableMotor(bool bit)
{
	if (bit != m_enableMotor)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_motorImpulse = scalar(0);
		m_enableMotor = bit;
	}
}

scalar b3RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

void b3RevoluteJoint::SetMotorSpeed(scalar speed)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_motorSpeed = speed;
}

scalar b3RevoluteJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

void b3RevoluteJoint::SetMaxMotorTorque(scalar torque)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_maxMotorTorque = torque;
}

void b3RevoluteJoint::Draw() const
{
	b3Transform xfA = GetFrameA();
	b3Draw_draw->DrawTransform(xfA);
	
	b3Transform xfB = GetFrameB();
	b3Draw_draw->DrawTransform(xfB);
}