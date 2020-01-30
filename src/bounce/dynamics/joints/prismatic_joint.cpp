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

#include <bounce/dynamics/joints/prismatic_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3PrismaticJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor, const b3Vec3& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);

	b3Quat qA = bodyA->GetOrientation();
	b3Quat qB = bodyB->GetOrientation();

	referenceRotation = b3Conjugate(qA) * qB;
}

b3PrismaticJoint::b3PrismaticJoint(const b3PrismaticJointDef* def)
{
	m_type = e_prismaticJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localXAxisA.Normalize();
	b3ComputeBasis(m_localXAxisA, m_localYAxisA, m_localZAxisA);

	m_referenceRotation = def->referenceRotation;

	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();
	m_limitImpulse = scalar(0);
	m_motorImpulse = scalar(0);

	m_lowerTranslation = def->lowerTranslation;
	m_upperTranslation = def->upperTranslation;
	m_maxMotorForce = def->maxMotorForce;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
	m_limitState = e_inactiveLimit;
}

void b3PrismaticJoint::InitializeConstraints(const b3SolverData* data)
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

	// Compute the effective masses.
	b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

	b3Vec3 d = xB + rB - xA - rA;

	scalar mA = m_mA, mB = m_mB;
	b3Mat33 iA = m_iA, iB = m_iB;

	// Compute motor Jacobian and effective mass.
	{
		m_axis = b3Mul(qA, m_localXAxisA);
		m_a1 = b3Cross(d + rA, m_axis);
		m_a2 = b3Cross(rB, m_axis);

		m_motorMass = mA + mB + b3Dot(iA * m_a1, m_a1) + b3Dot(iB * m_a2, m_a2);
		if (m_motorMass > scalar(0))
		{
			m_motorMass = scalar(1) / m_motorMass;
		}
	}

	// Prismatic constraint.
	{
		m_perp1 = b3Mul(qA, m_localYAxisA);
		m_s1 = b3Cross(d + rA, m_perp1);
		m_s2 = b3Cross(rB, m_perp1);

		m_perp2 = b3Mul(qA, m_localZAxisA);
		m_s3 = b3Cross(d + rA, m_perp2);
		m_s4 = b3Cross(rB, m_perp2);

		scalar k11 = mA + mB + b3Dot(m_s1, iA * m_s1) + b3Dot(m_s2, iB * m_s2);
		scalar k12 = b3Dot(m_s1, iA * m_s3) + b3Dot(m_s2, iB * m_s4);
		scalar k22 = mA + mB + b3Dot(m_s3, iA * m_s3) + b3Dot(m_s4, iB * m_s4);

		b3Mat22 K;
		K.x.x = k11;
		K.x.y = k12;
		K.y.x = k12;
		K.y.y = k22;

		m_linearMass = b3Inverse(K);
	}

	// Angular constraint.
	{
		m_angularMass = b3Inverse(iA + iB);
	}

	// Compute motor and limit terms.
	if (m_enableLimit)
	{
		scalar jointTranslation = b3Dot(m_axis, d);
		if (b3Abs(m_upperTranslation - m_lowerTranslation) < scalar(2) * B3_LINEAR_SLOP)
		{
			m_limitState = e_equalLimits;
		}
		else if (jointTranslation <= m_lowerTranslation)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_limitImpulse = scalar(0);
			}
		}
		else if (jointTranslation >= m_upperTranslation)
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
		m_limitImpulse = scalar(0);
	}

	if (m_enableMotor == false)
	{
		m_motorImpulse = scalar(0);
	}
}

void b3PrismaticJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Vec3 P = m_linearImpulse.x * m_perp1 + m_linearImpulse.y * m_perp2 + (m_motorImpulse + m_limitImpulse) * m_axis;
	b3Vec3 LA = m_linearImpulse.x * m_s1 + m_linearImpulse.y * m_s3 + m_angularImpulse + (m_motorImpulse + m_limitImpulse) * m_a1;
	b3Vec3 LB = m_linearImpulse.x * m_s2 + m_linearImpulse.y * m_s4 + m_angularImpulse + (m_motorImpulse + m_limitImpulse) * m_a2;

	vA -= m_mA * P;
	wA -= m_iA * LA;

	vB += m_mB * P;
	wB += m_iB * LB;

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3PrismaticJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	scalar mA = m_mA, mB = m_mB;
	b3Mat33 iA = m_iA, iB = m_iB;

	// Solve linear motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		scalar Cdot = b3Dot(m_axis, vB - vA) + b3Dot(m_a2, wB) - b3Dot(m_a1, wA);
		scalar impulse = m_motorMass * (m_motorSpeed - Cdot);
		scalar oldImpulse = m_motorImpulse;
		scalar maxImpulse = data->dt * m_maxMotorForce;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P = impulse * m_axis;
		b3Vec3 LA = impulse * m_a1;
		b3Vec3 LB = impulse * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve prismatic constraint in block form.
	{
		b3Vec2 Cdot;
		Cdot.x = b3Dot(m_perp1, vB - vA) + b3Dot(m_s2, wB) - b3Dot(m_s1, wA);
		Cdot.y = b3Dot(m_perp2, vB - vA) + b3Dot(m_s4, wB) - b3Dot(m_s3, wA);

		b3Vec2 impulse = m_linearMass * (-Cdot);

		m_linearImpulse += impulse;

		b3Vec3 P = impulse.x * m_perp1 + impulse.y * m_perp2;
		b3Vec3 LA = impulse.x * m_s1 + impulse.y * m_s3;
		b3Vec3 LB = impulse.x * m_s2 + impulse.y * m_s4;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve angular constraint in block form
	{
		b3Vec3 Cdot = wB - wA;
		b3Vec3 impulse = m_angularMass * (-Cdot);
		m_angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		scalar Cdot = b3Dot(m_axis, vB - vA) + b3Dot(m_a2, wB) - b3Dot(m_a1, wA);
		scalar impulse = m_motorMass * (-Cdot);
		scalar oldImpulse = m_limitImpulse;

		if (m_limitState == e_atLowerLimit)
		{
			m_limitImpulse = b3Max(m_limitImpulse + impulse, scalar(0));
		}
		else if (m_limitState == e_atUpperLimit)
		{
			m_limitImpulse = b3Min(m_limitImpulse + impulse, scalar(0));
		}

		impulse = m_limitImpulse - oldImpulse;

		b3Vec3 P = impulse * m_axis;
		b3Vec3 LA = impulse * m_a1;
		b3Vec3 LB = impulse * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3PrismaticJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	scalar mA = m_mA, mB = m_mB;

	// Compute fresh Jacobians
	b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);
	b3Vec3 d = xB + rB - xA - rA;

	b3Vec3 axis = b3Mul(qA, m_localXAxisA);
	b3Vec3 a1 = b3Cross(d + rA, axis);
	b3Vec3 a2 = b3Cross(rB, axis);

	b3Vec3 perp1 = b3Mul(qA, m_localYAxisA);
	b3Vec3 s1 = b3Cross(d + rA, perp1);
	b3Vec3 s2 = b3Cross(rB, perp1);

	b3Vec3 perp2 = b3Mul(qA, m_localZAxisA);
	b3Vec3 s3 = b3Cross(d + rA, perp2);
	b3Vec3 s4 = b3Cross(rB, perp2);

	scalar linearError = scalar(0);

	// Solve prismatic
	{
		b3Vec2 C;
		C.x = b3Dot(perp1, d);
		C.y = b3Dot(perp2, d);

		linearError += b3Abs(C.x) + b3Abs(C.y);

		scalar k11 = mA + mB + b3Dot(s1, iA * s1) + b3Dot(s2, iB * s2);
		scalar k12 = b3Dot(s1, iA * s3) + b3Dot(s2, iB * s4);
		scalar k22 = mA + mB + b3Dot(s3, iA * s3) + b3Dot(s4, iB * s4);

		b3Mat22 K;
		K.x.x = k11;
		K.x.y = k12;
		K.y.x = k12;
		K.y.y = k22;

		b3Mat22 linearMass = b3Inverse(K);

		b3Vec2 impulse = linearMass * (-C);

		b3Vec3 P = impulse.x * perp1 + impulse.y * perp2;
		b3Vec3 LA = impulse.x * s1 + impulse.y * s3;
		b3Vec3 LB = impulse.x * s2 + impulse.y * s4;

		xA -= mA * P;
		qA -= b3Derivative(qA, iA * LA);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * P;
		qB += b3Derivative(qB, iB * LB);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	scalar angularError = scalar(0);

	// Solve angular
	{
		b3Quat q1 = b3Conjugate(qA) * qB;
		b3Quat q2 = m_referenceRotation;

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
		b3Vec3 C = b3Mul(qA, -v);

		b3Mat33 mass = iA + iB;

		b3Vec3 impulse = mass.Solve(-C);

		qA -= b3Derivative(qA, iA * impulse);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * impulse);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	if (m_enableLimit)
	{
		scalar C = scalar(0);
		
		scalar translation = b3Dot(axis, d);
		if (b3Abs(m_upperTranslation - m_lowerTranslation) < scalar(2) * B3_LINEAR_SLOP)
		{
			// Prevent large angular corrections
			C = b3Clamp(translation, -B3_MAX_LINEAR_CORRECTION, B3_MAX_LINEAR_CORRECTION);
			linearError = b3Max(linearError, b3Abs(translation));
		}
		else if (translation <= m_lowerTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C = b3Clamp(translation - m_lowerTranslation + B3_LINEAR_SLOP, -B3_MAX_LINEAR_CORRECTION, scalar(0));
			linearError = b3Max(linearError, m_lowerTranslation - translation);
		}
		else if (translation >= m_upperTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C = b3Clamp(translation - m_upperTranslation - B3_LINEAR_SLOP, scalar(0), B3_MAX_LINEAR_CORRECTION);
			linearError = b3Max(linearError, translation - m_upperTranslation);
		}

		scalar limitMass = mA + mB + b3Dot(iA * a1, a1) + b3Dot(iB * a2, a2);
		limitMass = limitMass > scalar(0) ? scalar(1) / limitMass : scalar(0);

		scalar impulse = limitMass * -C;

		b3Vec3 P = impulse * axis;
		b3Vec3 LA = impulse * m_a1;
		b3Vec3 LB = impulse * m_a2;

		xA -= mA * P;
		qA -= b3Derivative(qA, iA * LA);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * P;
		qB += b3Derivative(qB, iB * LB);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->invInertias[m_indexA] = iA;
	
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && angularError <= B3_LINEAR_SLOP;
}

b3Vec3 b3PrismaticJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3PrismaticJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

scalar b3PrismaticJoint::GetJointTranslation() const
{
	b3Vec3 pA = GetBodyA()->GetWorldPoint(m_localAnchorA);
	b3Vec3 pB = GetBodyB()->GetWorldPoint(m_localAnchorB);
	b3Vec3 d = pB - pA;
	b3Vec3 axis = GetBodyA()->GetWorldVector(m_localXAxisA);

	scalar translation = b3Dot(d, axis);
	return translation;
}

scalar b3PrismaticJoint::GetJointSpeed() const
{
	const b3Body* bA = GetBodyA();
	const b3Body* bB = GetBodyB();

	b3Vec3 rA = b3Mul(bA->m_xf.rotation, m_localAnchorA - bA->m_sweep.localCenter);
	b3Vec3 rB = b3Mul(bB->m_xf.rotation, m_localAnchorB - bB->m_sweep.localCenter);

	b3Vec3 p1 = bA->m_sweep.worldCenter + rA;
	b3Vec3 p2 = bB->m_sweep.worldCenter + rB;

	b3Vec3 d = p2 - p1;

	b3Vec3 axis = b3Mul(bA->m_xf.rotation, m_localXAxisA);

	b3Vec3 vA = bA->m_linearVelocity;
	b3Vec3 vB = bB->m_linearVelocity;
	b3Vec3 wA = bA->m_angularVelocity;
	b3Vec3 wB = bB->m_angularVelocity;

	scalar speed = b3Dot(d, b3Cross(wA, axis)) + b3Dot(axis, vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA));
	return speed;
}

bool b3PrismaticJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b3PrismaticJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_enableLimit = flag;
		m_limitImpulse = scalar(0);
	}
}

scalar b3PrismaticJoint::GetLowerLimit() const
{
	return m_lowerTranslation;
}

scalar b3PrismaticJoint::GetUpperLimit() const
{
	return m_upperTranslation;
}

void b3PrismaticJoint::SetLimits(scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);
	if (lower != m_lowerTranslation || upper != m_upperTranslation)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
		m_limitImpulse = scalar(0);
	}
}

bool b3PrismaticJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b3PrismaticJoint::EnableMotor(bool flag)
{
	if (flag != m_enableMotor)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_enableMotor = flag;
	}
}

void b3PrismaticJoint::SetMotorSpeed(scalar speed)
{
	if (speed != m_motorSpeed)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_motorSpeed = speed;
	}
}

void b3PrismaticJoint::SetMaxMotorForce(scalar force)
{
	if (force != m_maxMotorForce)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_maxMotorForce = force;
	}
}

void b3PrismaticJoint::Draw() const
{
	b3Vec3 pA = GetAnchorA();
	b3Draw_draw->DrawPoint(pA, scalar(4), b3Color_red);

	b3Vec3 pB = GetAnchorB();
	b3Draw_draw->DrawPoint(pB, scalar(4), b3Color_green);

	b3Mat33 localAxesA(m_localXAxisA, m_localYAxisA, m_localZAxisA);
	b3Quat localRotationA = b3Mat33Quat(localAxesA);

	b3Transform xfA;
	xfA.translation = pA;
	xfA.rotation = GetBodyA()->GetWorldFrame(localRotationA);

	b3Draw_draw->DrawTransform(xfA);
}