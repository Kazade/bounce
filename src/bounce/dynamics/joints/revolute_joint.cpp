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

#include <bounce\dynamics\joints\revolute_joint.h>
#include <bounce\dynamics\body.h>
#include <bounce\common\draw.h>

// C1 = p2 - p1
// C2 = dot(u2, w1)
// C3 = dot(v2, w1)
// Cdot = dot(u2, omega1 x w1) + dot(w1, omega2 x u2)
// Cyclic identity:
// Cdot = dot(w1 x u2, omega1) + dot(u2 x w1, omega2) =
// dot(-u2 x w1, omega1) + dot(u2 x w1, omega2) 

// n1 = cross(u2, w1)^T
// n2 = cross(v2, w1)^T

// J = [-I  skew(r1) I -skew(r2)]
//     [0   -n1     0  n1]
//     [0   -n2     0  n2]

// W = [i1  0      0]
//     [0   m1 0   0]
//     [0   0  i2  0]
//     [0   0  0  m2]

void b3RevoluteJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor,
	float32 lower, float32 upper)
{
	b3Transform xf;
	xf.rotation.z = axis;
	xf.rotation.y = b3Perp(axis);
	xf.rotation.x = b3Cross(xf.rotation.y, axis);
	xf.position = anchor;

	bodyA = bA;
	bodyB = bB;
	localFrameA = bodyA->GetLocalFrame(xf);
	localFrameB = bodyB->GetLocalFrame(xf);
	lowerAngle = lower;
	upperAngle = upper;
	B3_ASSERT(lowerAngle <= upperAngle);
}

b3RevoluteJoint::b3RevoluteJoint(const b3RevoluteJointDef* def)
{
	m_type = e_revoluteJoint;
	m_localFrameA = def->localFrameA;
	m_localFrameB = def->localFrameB;
	m_enableLimit = def->enableLimit;
	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	B3_ASSERT(m_lowerAngle <= m_upperAngle);
	m_enableMotor = def->enableMotor;
	m_motorSpeed = def->motorSpeed;
	m_maxMotorTorque = def->maxMotorTorque;

	m_impulse[0] = 0.0f;
	m_impulse[1] = 0.0f;
	m_impulse[2] = 0.0f;
	m_impulse[3] = 0.0f;
	m_impulse[4] = 0.0f;
	m_nA.SetZero();
	m_nB.SetZero();

	m_motorImpulse = 0.0f;

	m_limitState = e_inactiveLimit;
	m_limitImpulse = 0.0f;
	m_limitAxis.SetZero();
}

void b3RevoluteJoint::InitializeConstraints(const b3SolverData* data)
{
	b3Body* m_bodyA = GetBodyA();
	b3Body* m_bodyB = GetBodyB();

	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_iA = m_bodyA->m_worldInvI;
	m_iB = m_bodyB->m_worldInvI;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	float32 mA = m_mA;
	b3Mat33 iA = m_iA;
	float32 mB = m_mB;
	b3Mat33 iB = m_iB;

	b3Transform xfA = m_bodyA->GetWorldFrame(m_localFrameA);
	b3Vec3 u1 = xfA.rotation.x;
	b3Vec3 v1 = xfA.rotation.y;
	b3Vec3 w1 = xfA.rotation.z;

	b3Transform xfB = m_bodyB->GetWorldFrame(m_localFrameB);
	b3Vec3 u2 = xfB.rotation.x;
	b3Vec3 v2 = xfB.rotation.y;
	b3Vec3 w2 = xfB.rotation.z;

	// Add motor constraint.
	if (m_enableMotor || m_enableLimit)
	{
		m_limitAxis = w1;
		float32 mass = b3Dot(m_limitAxis, (iA + iB) * m_limitAxis);
		m_motorMass = mass > 0.0f ? 1.0f / mass : 0.0f;
	}
		
	// Add limit constraint.
	if (m_enableLimit)
	{
		float32 cosine = b3Dot(u2, u1);
		float32 sine = b3Dot(u2, v1);
		float32 angle = atan2(sine, cosine);

		if (b3Abs(m_upperAngle - m_lowerAngle) < 2.0f * B3_ANGULAR_SLOP)
		{
			if (m_limitState != e_equalLimits)
			{
				m_limitState = e_equalLimits;
				m_limitImpulse = 0.0f;
			}
		}
		else if (angle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_limitImpulse = 0.0f;
			}
		}
		else if (angle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_limitState = e_atUpperLimit;
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

	// Add point-to-point and axes-to-axes constraints.
	{
		m_rA = b3Mul(qA, m_localFrameA.position - m_localCenterA);
		m_rB = b3Mul(qB, m_localFrameB.position - m_localCenterB);

		m_nA = b3Cross(u2, w1);
		m_nB = b3Cross(v2, w1);

		// Compute effective mass matrix.
		b3Vec3 rA = m_rA, rB = m_rB;
		b3Vec3 nA = m_nA, nB = m_nB;

		b3Mat33 M = b3Diagonal(mA + mB);
		b3Mat33 I = iA + iB;
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);

		// Identities and properties used:
		// 1x3 * 3x3 = transpose(3x3)*1x3 
		// 1x3 * 3x1 = dot(1x3, 3x1)
		// K = transpose(K) (SPD)
		b3Mat33 e11 = M + RA * iA * RAT + RB * iB * RBT;
		b3Vec3 e21 = b3Cross(rA, iA * -nA) + b3Cross(-rB, iB * nA);
		b3Vec3 e31 = b3Cross(rA, iA * -nB) + b3Cross(-rB, iB * nB);
		
		b3Vec3 e12 = e21;
		float32 e22 = b3Dot(nA, I * nA);
		float32 e32 = b3Dot(nA, I * nB);
		
		b3Vec3 e13 = e31;
		float32 e23 = e32;
		float32 e33 = b3Dot(nB, I * nB);

		b3Mat<5, 5> K;

		K(0, 0) = e11(0, 0);
		K(1, 0) = e11(1, 0);
		K(2, 0) = e11(2, 0);

		K(0, 1) = e11(0, 1);
		K(1, 1) = e11(1, 1);
		K(2, 1) = e11(2, 1);

		K(0, 2) = e11(0, 2);
		K(1, 2) = e11(1, 2);
		K(2, 2) = e11(2, 2);

		K(3, 0) = e21.x;
		K(3, 1) = e21.y;
		K(3, 2) = e21.z;

		K(4, 0) = e31.x;
		K(4, 1) = e31.y;
		K(4, 2) = e31.z;

		K(0, 3) = e12.x;
		K(1, 3) = e12.y;
		K(2, 3) = e12.z;

		K(0, 4) = e13.x;
		K(1, 4) = e13.y;
		K(2, 4) = e13.z;

		K(3, 3) = e22;
		K(3, 4) = e23;
		K(4, 3) = e32;
		K(4, 4) = e33;

		m_mass = K;
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
		b3Vec3 P = m_motorImpulse * m_limitAxis;
		wA -= m_iA * P;
		wB += m_iB * P;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b3Vec3 P = m_limitImpulse * m_limitAxis;
		wA -= m_iA * P;
		wB += m_iB * P;
	}

	{
		b3Vec3 P1(m_impulse[0], m_impulse[1], m_impulse[2]);
		b3Vec3 P2 = m_impulse[3] * m_nA + m_impulse[4] * m_nB;

		vA -= m_mA * P1;
		wA -= m_iA * (b3Cross(m_rA, P1) + P2);

		vB += m_mB * P1;
		wB += m_iB * (b3Cross(m_rB, P1) + P2);
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

	float32 mA = m_mA;
	b3Mat33 iA = m_iA;
	float32 mB = m_mB;
	b3Mat33 iB = m_iB;

	// Solve motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		float32 Cdot = b3Dot(m_limitAxis, wB - wA) - m_motorSpeed;
		float32 impulse = -m_motorMass * Cdot;
		float32 oldImpulse = m_motorImpulse;
		float32 maxImpulse = data->dt * m_maxMotorTorque;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P = impulse * m_limitAxis;

		wA -= iA * P;
		wB += iB * P;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 Cdot = b3Dot(m_limitAxis, wB - wA);
		float32 impulse = -m_motorMass * Cdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			float32 oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Max(m_limitImpulse + impulse, 0.0f);
			impulse = m_limitImpulse - oldImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			float32 oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Min(m_limitImpulse + impulse, 0.0f);
			impulse = m_limitImpulse - oldImpulse;
		}

		b3Vec3 P = impulse * m_limitAxis;

		wA -= iA * P;
		wB += iB * P;
	}

	// Solve point-to-point and axes-to-axes constraint.
	{
		b3Vec3 rA = m_rA;
		b3Vec3 rB = m_rB;
		b3Vec3 Cdot1 = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);

		b3Vec3 dw = wB - wA;
		
		b3Vec3 nA = m_nA;
		float32 Cdot2 = b3Dot(nA, dw);

		b3Vec3 nB = m_nB;
		float32 Cdot3 = b3Dot(nB, dw);

		b3Vec<5> Cdot;
		Cdot[0] = Cdot1.x;
		Cdot[1] = Cdot1.y;
		Cdot[2] = Cdot1.z;
		Cdot[3] = Cdot2;
		Cdot[4] = Cdot3;

		// Copy the effective mass so it can be destroyed in the 
		// linear solver.
		b3Mat<5, 5> mass = m_mass;
		b3Vec<5> impulse = -Cdot;
		if (b3Solve(impulse.e, mass.e, 5))
		{
			m_impulse += impulse;

			b3Vec3 P1(impulse[0], impulse[1], impulse[2]);
			b3Vec3 P2 = impulse[3] * nA + impulse[4] * nB;

			vA -= mA * P1;
			wA -= iA * (b3Cross(rA, P1) + P2);

			vB += mB * P1;
			wB += iB * (b3Cross(rB, P1) + P2);
		}
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

	float32 mA = m_mA;
	b3Mat33 iA = m_iA;
	float32 mB = m_mB;
	b3Mat33 iB = m_iB;

	// Solve limit constraint.
	float32 limitError = 0.0f;
	if (m_enableLimit)
	{
		b3Vec3 limitAxis = b3Mul(qA, m_localFrameA.rotation.z);
		
		// Compute fresh effective mass.
		float32 mass = b3Dot((iA + iB) * limitAxis, limitAxis);
		float32 limitMass = mass > 0.0f ? 1.0f / mass : 0.0f;

		// Compute joint angle.
		b3Vec3 u1 = b3Mul(qA, m_localFrameA.rotation.x);
		b3Vec3 v1 = b3Mul(qA, m_localFrameA.rotation.y);
		b3Vec3 u2 = b3Mul(qB, m_localFrameB.rotation.x);
		
		float32 cosine = b3Dot(u2, u1);
		float32 sine = b3Dot(u2, v1);
		float32 angle = atan2(sine, cosine);

		float32 limitImpulse = 0.0f;

		if (b3Abs(m_upperAngle - m_lowerAngle) < 2.0f * B3_ANGULAR_SLOP)
		{
			float32 C = angle - m_lowerAngle;
			limitError = b3Abs(C);
			
			// Prevent large corrections
			C = b3Clamp(C, -B3_MAX_ANGULAR_CORRECTION, B3_MAX_ANGULAR_CORRECTION);
			limitImpulse = -C * limitMass;
		}
		else if (angle <= m_lowerAngle)
		{
			float32 C = angle - m_lowerAngle;
			limitError = -C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C + B3_ANGULAR_SLOP, -B3_MAX_ANGULAR_CORRECTION, 0.0f);
			limitImpulse = -C * limitMass;
		}
		else if (angle >= m_upperAngle)
		{
			float32 C = angle - m_upperAngle;
			limitError = C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C - B3_ANGULAR_SLOP, 0.0f, B3_MAX_ANGULAR_CORRECTION);
			limitImpulse = -C * limitMass;
		}

		b3Vec3 P = limitImpulse * limitAxis;

		qA -= b3Derivative(qA, iA * P);
		qA.Normalize();

		qB += b3Derivative(qB, iB * P);
		qB.Normalize();
	}

	// Solve point-to-point and axes-to-axes constraints.
	float32 linearError = 0.0f;
	float32 angularError1 = 0.0f;
	float32 angularError2 = 0.0f;
	{
		b3Vec3 rA = b3Mul(qA, m_localFrameA.position - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localFrameB.position - m_localCenterB);
		b3Vec3 C1 = xB + rB - xA - rA;
		linearError = b3Length(C1);

		b3Vec3 w1 = b3Mul(qA, m_localFrameA.rotation.z);

		b3Vec3 u2 = b3Mul(qB, m_localFrameB.rotation.x);
		float32 C2 = b3Dot(u2, w1);
		angularError1 = b3Abs(C2);

		b3Vec3 v2 = b3Mul(qB, m_localFrameB.rotation.y);
		float32 C3 = b3Dot(v2, w1);
		angularError2 = b3Abs(C3);

		b3Vec<5> C;
		C[0] = C1.x;
		C[1] = C1.y;
		C[2] = C1.z;
		C[3] = C2;
		C[4] = C3;

		// Compute effective mass matrix.
		b3Vec3 nA = b3Cross(u2, w1);
		b3Vec3 nB = b3Cross(v2, w1);

		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Diagonal(mA + mB);
		b3Mat33 I = iA + iB;

		b3Mat33 e11 = M + RA * iA * RAT + RB * iB * RBT;
		b3Vec3 e21 = b3Cross(rA, iA * -nA) + b3Cross(-rB, iB * nA);
		b3Vec3 e31 = b3Cross(rA, iA * -nB) + b3Cross(-rB, iB * nB);
		
		b3Vec3 e12 = e21;
		float32 e22 = b3Dot(nA, I * nA);
		float32 e32 = b3Dot(nA, I * nB);

		b3Vec3 e13 = e31;
		float32 e23 = e32;
		float32 e33 = b3Dot(nB, I * nB);

		b3Mat<5, 5> K;

		K(0, 0) = e11(0, 0);
		K(1, 0) = e11(1, 0);
		K(2, 0) = e11(2, 0);

		K(0, 1) = e11(0, 1);
		K(1, 1) = e11(1, 1);
		K(2, 1) = e11(2, 1);

		K(0, 2) = e11(0, 2);
		K(1, 2) = e11(1, 2);
		K(2, 2) = e11(2, 2);

		K(3, 0) = e21.x;
		K(3, 1) = e21.y;
		K(3, 2) = e21.z;

		K(4, 0) = e31.x;
		K(4, 1) = e31.y;
		K(4, 2) = e31.z;

		K(0, 3) = e12.x;
		K(1, 3) = e12.y;
		K(2, 3) = e12.z;

		K(0, 4) = e13.x;
		K(1, 4) = e13.y;
		K(2, 4) = e13.z;

		K(3, 3) = e22;
		K(3, 4) = e23;
		K(4, 3) = e32;
		K(4, 4) = e33;

		b3Vec<5> impulse = -C;
		if (b3Solve(impulse.e, K.e, 5))
		{
			b3Vec3 P1(impulse[0], impulse[1], impulse[2]);
			b3Vec3 P2 = impulse[3] * nA + impulse[4] * nB;

			xA -= mA * P1;
			qA -= b3Derivative(qA, iA * (b3Cross(rA, P1) + P2));
			qA.Normalize();

			xB += mB * P1;
			qB += b3Derivative(qB, iB * (b3Cross(rB, P1) + P2));
			qB.Normalize();
		}
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;

	return linearError <= B3_LINEAR_SLOP &&
		angularError1 <= B3_ANGULAR_SLOP &&
		angularError2 <= B3_ANGULAR_SLOP &&
		limitError <= B3_ANGULAR_SLOP;
}

const b3Transform& b3RevoluteJoint::GetFrameA() const
{
	return m_localFrameA;
}

void b3RevoluteJoint::SetFrameA(const b3Transform& frame)
{
	m_localFrameA = frame;
}

const b3Transform& b3RevoluteJoint::GetFrameB() const
{
	return m_localFrameB;
}

void b3RevoluteJoint::SetFrameB(const b3Transform& frame)
{
	m_localFrameB = frame;
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
		m_limitImpulse = 0.0f;
		m_limitState = e_inactiveLimit;
		m_enableLimit = bit;
	}
}

float32 b3RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

float32 b3RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void b3RevoluteJoint::SetLimits(float32 lower, float32 upper)
{
	B3_ASSERT(lower <= upper);
	if (lower != m_lowerAngle || upper != m_upperAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = 0.0f;
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
		m_motorImpulse = 0.0f;
		m_enableMotor = bit;
	}
}

float32 b3RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

void b3RevoluteJoint::SetMotorSpeed(float32 speed)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_motorSpeed = speed;
}

float32 b3RevoluteJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

void b3RevoluteJoint::SetMaxMotorTorque(float32 torque)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_maxMotorTorque = torque;
}

void b3RevoluteJoint::Draw(b3Draw* b3Draw) const
{
	b3Transform xfA = GetBodyA()->GetWorldFrame(m_localFrameA);
	b3Draw->DrawTransform(xfA);
	b3Transform xfB = GetBodyB()->GetWorldFrame(m_localFrameB);
	b3Draw->DrawTransform(xfB);
}
