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

#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

/*
P = [0 1 0 0]
	[0 0 1 0]
	[0 0 0 1]

q = conj(q1) * q2

C = P * q
C' = P * q'

q' = 
conj(q1)' * q2 + conj(q1) * q2' =
conj(q2') * q2 + conj(q1) * q2'

J1 = -0.5 * Q(conj(q1)) * P(q2)
J2 =  0.5 * Q(conj(q1)) * P(q2)

J1 = P * J1 * P^T
J2 = P * J2 * P^T
*/

static b3Mat44 iQ_mat(const b3Quat& q)
{
	b3Mat44 Q;
	Q.x = b3Vec4(q.w, q.x, q.y, q.z);
	Q.y = b3Vec4(-q.x, q.w, q.z, -q.y);
	Q.z = b3Vec4(-q.y, -q.z, q.w, q.x);
	Q.w = b3Vec4(-q.z, q.y, -q.x, q.w);
	return Q;
}

static b3Mat44 iP_mat(const b3Quat& q)
{
	b3Mat44 P;
	P.x = b3Vec4(q.w, q.x, q.y, q.z);
	P.y = b3Vec4(-q.x, q.w, -q.z, q.y);
	P.z = b3Vec4(-q.y, q.z, q.w, -q.x);
	P.w = b3Vec4(-q.z, -q.y, q.x, q.w);
	return P;
}

static b3Mat34 P_mat()
{
	b3Mat34 P;
	P.x = b3Vec3(0.0f, 0.0f, 0.0f);
	P.y = b3Vec3(1.0f, 0.0f, 0.0f);
	P.z = b3Vec3(0.0f, 1.0f, 0.0f);
	P.w = b3Vec3(0.0f, 0.0f, 1.0f);
	return P;
}

static b3Mat34 P_lock_mat()
{
	b3Mat34 P;
	P.x = b3Vec3(0.0f, 0.0f, 0.0f);
	P.y = b3Vec3(1.0f, 0.0f, 0.0f);
	P.z = b3Vec3(0.0f, 1.0f, 0.0f);
	P.w = b3Vec3(0.0f, 0.0f, 1.0f);
	return P;
}

static b3Vec4 q_to_v(const b3Quat& q)
{
	return b3Vec4(q.w, q.x, q.y, q.z);
}

static const b3Mat34 P = P_mat();
static const b3Mat43 PT = b3Transpose(P);
static const b3Mat34 P_lock = P_lock_mat();

void b3WeldJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);

	b3Quat qA = bodyA->GetOrientation();
	b3Quat qB = bodyB->GetOrientation();

	referenceRotation = b3Conjugate(qA) * qB;
}

b3WeldJoint::b3WeldJoint(const b3WeldJointDef* def)
{
	m_type = e_weldJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceRotation = def->referenceRotation;
	m_impulse.SetZero();
	m_axisImpulse.SetZero();
}

void b3WeldJoint::InitializeConstraints(const b3SolverData* data)
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

	{
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

	{
		b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(qA) * qB;

		m_J1 = -0.5f * P_lock * iQ_mat(b3Conjugate(qA)) * iP_mat(qB) * PT;
		m_J2 =  0.5f * P_lock * iQ_mat(b3Conjugate(qA)) * iP_mat(qB) * PT;

		m_J1T = b3Transpose(m_J1);
		m_J2T = b3Transpose(m_J2);

		m_K = m_J1 * m_iA * m_J1T + m_J2 * m_iB * m_J2T;
	}
}

void b3WeldJoint::WarmStart(const b3SolverData* data)
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

void b3WeldJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_mass.Solve(-Cdot);

		m_impulse += impulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}

	{
		b3Vec3 Cdot = m_J1 * wA + m_J2 * wB;
		b3Vec3 impulse = m_K.Solve(-Cdot);
		
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

bool b3WeldJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;

	float32 linearError = 0.0f;

	{
		// Compute effective mass
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 M = b3Diagonal(m_mA + m_mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);

		b3Mat33 mass = M + RA * m_iA * RAT + RB * m_iB * RBT;

		b3Vec3 C = xB + rB - xA - rA;
		b3Vec3 impulse = mass.Solve(-C);

		xA -= m_mA * impulse;
		qA -= b3Derivative(qA, b3Mul(m_iA, b3Cross(rA, impulse)));
		qA.Normalize();

		xB += m_mB * impulse;
		qB += b3Derivative(qB, b3Mul(m_iB, b3Cross(rB, impulse)));
		qB.Normalize();

		linearError += b3Length(C);
	}

	float32 angularError = 0.0f;

	{		
		b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(qA) * qB;
		b3Vec4 dq_v = q_to_v(dq);

		b3Vec3 C = P * dq_v;

		angularError += b3Length(C);

		b3Mat33 J1 = -0.5f * P_lock * iQ_mat(b3Conjugate(qA)) * iP_mat(qB) * PT;
		b3Mat33 J2 = 0.5f * P_lock * iQ_mat(b3Conjugate(qA)) * iP_mat(qB) * PT;

		b3Mat33 J1T = b3Transpose(J1);
		b3Mat33 J2T = b3Transpose(J2);

		b3Mat33 mass = J1 * m_iA * J1T + J2 * m_iB * J2T;
		b3Vec3 impulse = mass.Solve(-C);

		b3Vec3 P1 = J1T * impulse;
		b3Vec3 P2 = J2T * impulse;

		qA += b3Derivative(qA, m_iA * P1);
		qA.Normalize();

		qB += b3Derivative(qB, m_iB * P2);
		qB.Normalize();
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;

	return linearError <= B3_LINEAR_SLOP && angularError <= B3_ANGULAR_SLOP;
}

b3Vec3 b3WeldJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3WeldJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

void b3WeldJoint::Draw() const
{
	b3Vec3 a = GetAnchorA();
	b3Draw_draw->DrawPoint(a, 4.0f, b3Color_red);
	
	b3Vec3 b = GetAnchorB();
	b3Draw_draw->DrawPoint(b, 4.0f, b3Color_green);
	
	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}