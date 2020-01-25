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

#include <bounce/dynamics/joints/wheel_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

void b3WheelJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor, const b3Vec3& axisA, const b3Vec3& axisB)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axisA);
	localAxisB = bodyB->GetLocalVector(axisB);
}

b3WheelJoint::b3WheelJoint(const b3WheelJointDef* def)
{
	m_type = e_wheelJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localXAxisA.Normalize();
	b3ComputeBasis(m_localXAxisA, m_localYAxisA, m_localZAxisA);

	m_localXAxisB = def->localAxisB;
	m_localXAxisB.Normalize();
	
	b3Vec3 axisA = def->bodyA->GetWorldVector(m_localXAxisA);
	b3Vec3 axisB = def->bodyB->GetWorldVector(m_localXAxisB);

	b3Vec3 u = b3Cross(axisA, axisB);

	scalar x = b3Dot(axisA, axisB);
	scalar y = b3Length(u);

	m_referenceCosine = x;
	m_referenceAngle = atan2(y, x);

	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;

	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_linearImpulse.SetZero();
	m_angularImpulse = scalar(0);
	m_motorImpulse = scalar(0);
	m_springMass = scalar(0);
	m_springImpulse = scalar(0);
}

void b3WheelJoint::InitializeConstraints(const b3SolverData* data)
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

	// Spring constraint.
	m_springMass = scalar(0);
	m_bias = scalar(0);
	m_gamma = scalar(0);
	if (m_frequencyHz > scalar(0))
	{
		m_axisA = b3Mul(qA, m_localXAxisA);
		m_a1 = b3Cross(d + rA, m_axisA);
		m_a2 = b3Cross(rB, m_axisA);

		scalar invMass = mA + mB + b3Dot(iA * m_a1, m_a1) + b3Dot(iB * m_a2, m_a2);
		if (invMass > scalar(0))
		{
			m_springMass = scalar(1) / invMass;

			scalar C = b3Dot(d, m_axisA);

			// Frequency
			scalar omega = scalar(2) * B3_PI * m_frequencyHz;

			// Damping coefficient
			scalar d = scalar(2) * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			scalar k = m_springMass * omega * omega;

			// magic formulas
			scalar h = data->dt;
			m_gamma = h * (d + h * k);
			if (m_gamma > scalar(0))
			{
				m_gamma = scalar(1) / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > scalar(0))
			{
				m_springMass = scalar(1) / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = scalar(0);
	}

	if (m_enableMotor)
	{
		m_axisB = b3Mul(qB, m_localXAxisB);

		m_motorMass = b3Dot((iA + iB) * m_axisB, m_axisB);
		if (m_motorMass > scalar(0))
		{
			m_motorMass = scalar(1) / m_motorMass;
		}
	}
	else
	{
		m_motorImpulse = scalar(0);
	}

	// Angular constraint
	{
		b3Vec3 axisA = b3Mul(qA, m_localXAxisA);
		b3Vec3 axisB = b3Mul(qB, m_localXAxisB);

		m_u = b3Cross(axisA, axisB);

		m_angularMass = b3Dot((iA + iB) * m_u, m_u);
		if (m_angularMass > scalar(0))
		{
			m_angularMass = scalar(1) / m_angularMass;
		}
	}
}

void b3WheelJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;
	
	{
		b3Vec3 P = m_linearImpulse.x * m_perp1 + m_linearImpulse.y * m_perp2 + m_springImpulse * m_axisA;
		b3Vec3 LA = m_linearImpulse.x * m_s1 + m_linearImpulse.y * m_s3 + m_springImpulse * m_a1 + m_motorImpulse * m_axisB;
		b3Vec3 LB = m_linearImpulse.x * m_s2 + m_linearImpulse.y * m_s4 + m_springImpulse * m_a2 + m_motorImpulse * m_axisB;

		vA -= m_mA * P;
		wA -= m_iA * LA;

		vB += m_mB * P;
		wB += m_iB * LB;
	}

	{
		b3Vec3 L = m_angularImpulse * m_u;

		wA += m_iA * L;
		wB -= m_iB * L;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3WheelJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	scalar mA = m_mA, mB = m_mB;
	b3Mat33 iA = m_iA, iB = m_iB;

	// Solve spring constraint.
	{
		scalar Cdot = b3Dot(m_axisA, vB - vA) + b3Dot(m_a2, wB) - b3Dot(m_a1, wA);
		scalar impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		b3Vec3 P = impulse * m_axisA;
		b3Vec3 LA = impulse * m_a1;
		b3Vec3 LB = impulse * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve rotational motor constraint in block form
	if (m_enableMotor)
	{
		scalar dw = b3Dot(wB - wA, m_axisB);
		scalar Cdot = dw - m_motorSpeed;
		scalar impulse = -m_motorMass * Cdot;
		scalar oldImpulse = m_motorImpulse;
		scalar maxImpulse = data->dt * m_maxMotorTorque;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P = m_axisB * impulse;

		wA -= iA * P;
		wB += iB * P;
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

	// Solve angular constraint.
	{
		scalar Cdot = b3Dot(wA - wB, m_u);
		scalar impulse = -m_angularMass * Cdot;

		m_angularImpulse += impulse;
		
		b3Vec3 L = impulse * m_u;

		wA += iA * L;
		wB -= iB * L;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3WheelJoint::SolvePositionConstraints(const b3SolverData* data)
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

	b3Vec3 axisA = b3Mul(qA, m_localXAxisA);
	b3Vec3 a1 = b3Cross(d + rA, axisA);
	b3Vec3 a2 = b3Cross(rB, axisA);

	b3Vec3 perp1 = b3Mul(qA, m_localYAxisA);
	b3Vec3 s1 = b3Cross(d + rA, perp1);
	b3Vec3 s2 = b3Cross(rB, perp1);

	b3Vec3 perp2 = b3Mul(qA, m_localZAxisA);
	b3Vec3 s3 = b3Cross(d + rA, perp2);
	b3Vec3 s4 = b3Cross(rB, perp2);

	b3Vec3 axisB = b3Mul(qB, m_localXAxisB);

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
		b3Vec3 u = b3Cross(axisA, axisB);

		scalar cosine = b3Dot(axisA, axisB);
		
		scalar C = cosine - m_referenceCosine;

		scalar sine = b3Length(u);
		scalar angle = atan2(sine, cosine);
		scalar angleC = angle - m_referenceAngle;
		
		angularError += b3Abs(angleC);

		scalar mass = b3Dot((iA + iB) * u, u);
		if (mass > scalar(0))
		{
			mass = scalar(1) / mass;
		}

		scalar impulse = -mass * C;

		b3Vec3 P = impulse * u;

		qA += b3Derivative(qA, iA * P);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB -= b3Derivative(qB, iB * P);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->invInertias[m_indexA] = iA;

	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && angularError <= B3_ANGULAR_SLOP;
}

b3Vec3 b3WheelJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3WheelJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

scalar b3WheelJoint::GetJointTranslation() const
{
	b3Vec3 pA = GetBodyA()->GetWorldPoint(m_localAnchorA);
	b3Vec3 pB = GetBodyB()->GetWorldPoint(m_localAnchorB);
	b3Vec3 d = pB - pA;
	b3Vec3 axis = GetBodyA()->GetWorldVector(m_localXAxisA);

	scalar translation = b3Dot(d, axis);
	return translation;
}

scalar b3WheelJoint::GetJointLinearSpeed() const
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

b3Quat b3WheelJoint::GetJointRotation() const
{
	const b3Body* bA = GetBodyA();
	const b3Body* bB = GetBodyB();
	return b3Conjugate(bA->GetOrientation()) * bB->GetOrientation();
}

scalar b3WheelJoint::GetJointAngularSpeed() const
{
	const b3Body* bA = GetBodyA();
	const b3Body* bB = GetBodyB();
	return b3Dot(bB->GetAngularVelocity() - bA->GetAngularVelocity(), bB->GetWorldVector(m_localXAxisB));
}

bool b3WheelJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b3WheelJoint::EnableMotor(bool flag)
{
	if (flag != m_enableMotor)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_enableMotor = flag;
	}
}

void b3WheelJoint::SetMotorSpeed(scalar speed)
{
	if (speed != m_motorSpeed)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_motorSpeed = speed;
	}
}

void b3WheelJoint::SetMaxMotorTorque(scalar torque)
{
	if (torque != m_maxMotorTorque)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_maxMotorTorque = torque;
	}
}

void b3WheelJoint::Draw() const
{
	b3Vec3 pA = GetAnchorA();
	b3Draw_draw->DrawPoint(pA, scalar(4), b3Color_red);

	b3Vec3 pB = GetAnchorB();
	b3Draw_draw->DrawPoint(pB, scalar(4), b3Color_green);

	const b3Body* bA = GetBodyA();
	const b3Body* bB = GetBodyB();

	b3Mat33 localAxes(m_localXAxisA, m_localYAxisA, m_localZAxisA);
	b3Quat localRotationA = b3Mat33Quat(localAxes);

	b3Transform xfA;
	xfA.translation = pA;
	xfA.rotation = bA->GetWorldFrame(localRotationA);

	b3Draw_draw->DrawTransform(xfA);

	b3Vec3 axisB = bB->GetWorldVector(m_localXAxisB);
	
	b3Draw_draw->DrawSegment(pB, pB + axisB, b3Color_white);
}