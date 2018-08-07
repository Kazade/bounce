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

#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

/*

Algebra:

Q(p) * P(q) = P(q) * Q(p)
q' = 0.5 * w * q

P = [0 1 0 0]
	[0 0 1 0]
	[0 0 0 1]

Hinge projection matrix:

P_hin = [x^T] * P = [0 1 0 0]
		[y^T]		[0 0 1 0]

Constraint:

q = conj(q1) * q2

C = P_hin * q

Chain rule:

q' = 
conj(q1)' * q2 + conj(q1) * q2' =
conj(q1') * q2 + conj(q1) * q2'

1st term:

conj(q1') * q2 =
0.5 * conj(w1 * q1) * w2 =
0.5 * conj(q1) * conj(w1) * q2 =
0.5 * conj(q1) * -w1 * q2 =
-0.5 * conj(q1) * w1 * q2 =
-0.5 * Q(conj(q1)) * P(q2) * Q(w1)

J1 = -0.5 * Q(conj(qA)) * P(qB)

2nd term:

conj(q1) * q2' =
0.5 * conj(q1) * w2 * q2 =
0.5 * Q(conj(q1)) * Q(w2) * Q(q2) =
0.5 * Q(conj(q1)) * P(q2) * Q(w2)

J2 = 0.5 * Q(conj(q1)) * P(q2)

C' = P_hin * q' =
P_hin * (J1 * P^T * w1 + J2 * P^T * w2) =
P_hin * J1 * P^T * w1 + P_hin * J2 * P^T * w2

New Jacobians:

J1 = P_hin * J1 * P^T
J2 = P_hin * J2 * P^T

Limit constraint:

q = conj(q1) * q2

C = 2 * atan(q.z / q.w)

Chain rule:

f( g( q(t) ) ) = 2 * atan( g( q(t) ) )
g( q(t) ) = q.z / q.w

df / dt = del_f / del_g * del_g / del_q * dq / dt

del_f / del_g = 
1 / (g^2 + 1) = 
1 / ((q.z / q.w)^2 + 1) = 
q.w^2 / (q.w^2 + q.z^2) ~
q.w^2

del_g / del_q = 
[del_g / del_w | del_g / del_x | del_g / del_y | del_g / del_z] = 
[-q.z/q.w^2  0  0  q.w/q.w^2] =
[-q.z/q.w^2  0  0  1/q.w] =
1 / q.w^2 * [-q.z  0  0  q.w]

df / dt = 
q.w^2 * 1 / q.w^2 * [-q.z  0  0  q.w] * dq / dt =
[-q.z  0  0  q.w] * dq / dt

P_lim = [-q.z  0  0  q.w]

C' = P_lim * (P_hinge * q') - target_speed

*/

static B3_FORCE_INLINE b3Mat44 iQ_mat(const b3Quat& q)
{
	b3Mat44 Q;
	Q.x = b3Vec4(q.w, q.x, q.y, q.z);
	Q.y = b3Vec4(-q.x, q.w, q.z, -q.y);
	Q.z = b3Vec4(-q.y, -q.z, q.w, q.x);
	Q.w = b3Vec4(-q.z, q.y, -q.x, q.w);
	return Q;
}

static B3_FORCE_INLINE b3Mat44 iP_mat(const b3Quat& q)
{
	b3Mat44 P;
	P.x = b3Vec4(q.w, q.x, q.y, q.z);
	P.y = b3Vec4(-q.x, q.w, -q.z, q.y);
	P.z = b3Vec4(-q.y, q.z, q.w, -q.x);
	P.w = b3Vec4(-q.z, -q.y, q.x, q.w);
	return P;
}

static B3_FORCE_INLINE b3Mat34 P_mat()
{
	b3Mat34 P;
	P.x = b3Vec3(0.0f, 0.0f, 0.0f);
	P.y = b3Vec3(1.0f, 0.0f, 0.0f);
	P.z = b3Vec3(0.0f, 1.0f, 0.0f);
	P.w = b3Vec3(0.0f, 0.0f, 1.0f);
	return P;
}

static B3_FORCE_INLINE b3Mat24 P_hinge_mat()
{
	b3Mat24 P;
	P.x = b3Vec2(0.0f, 0.0f);
	P.y = b3Vec2(1.0f, 0.0f);
	P.z = b3Vec2(0.0f, 1.0f);
	P.w = b3Vec2(0.0f, 0.0f);
	return P;
}

// 1x4 
static B3_FORCE_INLINE b3Vec4 P_hinge_limit_mat(const b3Quat& q)
{
	return b3Vec4(-q.z, 0.0f, 0.0f, q.w);
}

// 4x1 
static B3_FORCE_INLINE b3Vec4 q_to_v(const b3Quat& q)
{
	return b3Vec4(q.w, q.x, q.y, q.z);
}

static const b3Mat34 P = P_mat();
static const b3Mat43 PT = b3Transpose(P);
static const b3Mat24 P_hinge = P_hinge_mat();

void b3RevoluteJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor,
	float32 lower, float32 upper)
{
	B3_ASSERT(b3Length(axis) > B3_EPSILON);
	B3_ASSERT(lowerAngle <= upperAngle);

	b3Mat33 rotation;
	rotation.z = axis;
	rotation.y = b3Perp(axis);
	rotation.x = b3Cross(rotation.y, axis);

	b3Quat q = b3Mat33Quat(rotation);
	float32 len = q.Normalize();
	B3_ASSERT(len > B3_EPSILON);

	b3Quat qA = bA->GetOrientation();
	b3Quat qB = bB->GetOrientation();
	
	bodyA = bA;
	bodyB = bB;
	
	localAnchorA = bA->GetLocalPoint(anchor);
	localRotationA = b3Conjugate(qA) * q;

	localAnchorB = bB->GetLocalPoint(anchor);
	localRotationB = b3Conjugate(qB) * q;

	referenceRotation = b3Conjugate(qA * localRotationA) * qB * localRotationB;
	
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
	
	m_motorImpulse = 0.0f;

	m_limitState = e_inactiveLimit;
	m_limitImpulse = 0.0f;
	
	m_impulse.SetZero();

	m_axisImpulse.SetZero();
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

	float32 mA = m_mA;
	b3Mat33 iA = m_iA;
	float32 mB = m_mB;
	b3Mat33 iB = m_iB;

	// Joint rotation
	b3Quat fA = qA * m_localRotationA;
	b3Quat fB = qB * m_localRotationB;
	b3Quat q = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

	// Add motor constraint.
	if (m_enableMotor || m_enableLimit)
	{
		b3Vec4 P_hinge_limit = P_hinge_limit_mat(q);

		b3Mat44 G1 = -0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);
		b3Mat44 G2 = 0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);

		b3Vec3 J1 = P_hinge_limit * G1 * PT;
		b3Vec3 J2 = P_hinge_limit * G2 * PT;
		
		b3Vec3 J1T = J1;
		b3Vec3 J2T = J2;

		m_motor_J1 = J1;
		m_motor_J2 = J2;

		float32 K = J1 * iA * J1T + J2 * iB * J2T;
		m_motorMass = K > 0.0f ? 1.0f / K : 0.0f;
	}
		
	// Add limit constraint.
	if (m_enableLimit)
	{
		// Compute joint angle
		float32 angle = 2.0f * atan2(q.z, q.w);
		
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

	// Add point-to-point constraints.
	{
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Diagonal(mA + mB);

		m_mass = M + RA * iA * RAT + RB * iB * RBT;
	}

	// Add hinge constraints.
	{
		b3Mat44 G1 = -0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);
		b3Mat44 G2 = 0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);

		b3Mat23 J1 = P_hinge * G1 * PT;
		b3Mat23 J2 = P_hinge * G2 * PT;
		
		b3Mat32 J1T = b3Transpose(J1);
		b3Mat32 J2T = b3Transpose(J2);
		
		m_J1 = J1;
		m_J2 = J2;

		m_J1T = J1T;
		m_J2T = J2T;

		b3Mat22 K = J1 * iA * J1T + J2 * iB * J2T;
		m_K = b3Inverse(K);
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
		b3Vec3 P1 = m_motor_J1 * m_motorImpulse;
		b3Vec3 P2 = m_motor_J2 * m_motorImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b3Vec3 P1 = m_motor_J1 * m_limitImpulse;
		b3Vec3 P2 = m_motor_J2 * m_limitImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	{
		vA -= m_mA * m_impulse;
		wA -= m_iA * b3Cross(m_rA, m_impulse);

		vB += m_mB * m_impulse;
		wB += m_iB * b3Cross(m_rB, m_impulse);
	}

	{
		b3Vec3 P1 = m_J1T * m_axisImpulse;
		b3Vec3 P2 = m_J2T * m_axisImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
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
		float32 dw = m_motor_J1 * wA + m_motor_J2 * wB;
		float32 Cdot = dw - m_motorSpeed;
		float32 impulse = -m_motorMass * Cdot;
		float32 oldImpulse = m_motorImpulse;
		float32 maxImpulse = data->dt * m_maxMotorTorque;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P1 = m_motor_J1 * impulse;
		b3Vec3 P2 = m_motor_J2 * impulse;

		wA += iA * P1;
		wB += iB * P2;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		float32 Cdot = m_motor_J1 * wA + m_motor_J2 * wB;
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

		b3Vec3 P1 = m_motor_J1 * impulse;
		b3Vec3 P2 = m_motor_J2 * impulse;

		wA += iA * P1;
		wB += iB * P2;
	}

	// Solve point-to-point constraints.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_mass.Solve(-Cdot);

		m_impulse += impulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}

	// Solve axes-to-axes constraint.
	{
		b3Vec2 Cdot = m_J1 * wA + m_J2 * wB;
		b3Vec2 impulse = m_K * -Cdot;

		m_axisImpulse += impulse;

		b3Vec3 P1 = m_J1T * impulse;
		b3Vec3 P2 = m_J2T * impulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
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

	float32 mA = m_mA;
	float32 mB = m_mB;

	// Solve limit constraint.
	float32 limitError = 0.0f;
	
	if (m_enableLimit)
	{
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;
		b3Quat q = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		b3Vec4 P_hinge_limit = P_hinge_limit_mat(q);

		b3Mat44 G1 = -0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);
		b3Mat44 G2 =  0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);

		b3Vec3 J1 = P_hinge_limit * G1 * PT;
		b3Vec3 J2 = P_hinge_limit * G2 * PT;

		b3Vec3 J1T = J1;
		b3Vec3 J2T = J2;
		
		float32 K = J1 * iA * J1T + J2 * iB * J2T;
		float32 limitMass = K > 0.0f ? 1.0f / K : 0.0f;
		
		float32 limitImpulse = 0.0f;

		float32 angle = 2.0f * atan2(q.z, q.w);
		
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

		b3Vec3 P1 = J1T * limitImpulse;
		b3Vec3 P2 = J2T * limitImpulse;

		qA += b3Derivative(qA, iA * P1);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * P2);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}
	
	// Solve point-to-point constraints.
	float32 linearError = 0.0f;
	
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

	// Solve hinge constraints.
	float32 angularError = 0.0f;

	{
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;
		b3Quat q = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		b3Vec2 C = P_hinge * q_to_v(q);
		
		angularError += b3Length(C);
		
		// Compute effective mass
		b3Mat44 G1 = -0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);
		b3Mat44 G2 = 0.5f * iQ_mat(b3Conjugate(fA)) * iP_mat(fB);

		b3Mat23 J1 = P_hinge * G1 * PT;
		b3Mat23 J2 = P_hinge * G2 * PT;
		
		b3Mat32 J1T = b3Transpose(J1);
		b3Mat32 J2T = b3Transpose(J2);
		
		b3Mat22 mass = J1 * iA * J1T + J2 * iB * J2T;
		b3Vec2 impulse = mass.Solve(-C);

		b3Vec3 P1 = J1T * impulse;
		b3Vec3 P2 = J2T * impulse;

		qA += b3Derivative(qA, iA * P1);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * P2);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && 
		angularError <= B3_ANGULAR_SLOP &&
		limitError <= B3_ANGULAR_SLOP;
}

b3Transform b3RevoluteJoint::GetFrameA() const
{
	b3Transform xf(m_localRotationA, m_localAnchorA);
	return GetBodyA()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetFrameB() const
{
	b3Transform xf(m_localRotationB, m_localAnchorB);
	return GetBodyB()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetLocalFrameA() const
{
	return b3Transform(m_localRotationA, m_localAnchorA);
}

b3Transform b3RevoluteJoint::GetLocalFrameB() const
{
	return b3Transform(m_localRotationB, m_localAnchorB);
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

void b3RevoluteJoint::Draw() const
{
	b3Transform xfA = GetFrameA();
	b3Draw_draw->DrawTransform(xfA);
	
	b3Transform xfB = GetFrameB();
	b3Draw_draw->DrawTransform(xfB);
}