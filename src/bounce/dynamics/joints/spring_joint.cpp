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

#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

// C = ||x2 + r2 - x1 - r1|| - length
// Cdot = dot(n, v2 + w2 x r2 - v1 - w1 x r1)
// J = [-n^T -cross(n, r1)^T n^T cross(n, r2)^T]

void b3SpringJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchorA, const b3Vec3& anchorB)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchorA);
	localAnchorB = bodyB->GetLocalPoint(anchorB);
	length = b3Distance(anchorA, anchorB);
}

b3SpringJoint::b3SpringJoint(const b3SpringJointDef* def) 
{
	m_type = e_springJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_length = def->length;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;
	m_impulse = 0.0f;
}

b3Vec3 b3SpringJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3SpringJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

const b3Vec3& b3SpringJoint::GetLocalAnchorA() const
{
	return m_localAnchorA;
}

const b3Vec3& b3SpringJoint::GetLocalAnchorB() const
{
	return m_localAnchorB;
}

float32 b3SpringJoint::GetLength() const
{
	return m_length;
}

void b3SpringJoint::SetLength(float32 length)
{
	m_length = length;
}

float32 b3SpringJoint::GetFrequency() const
{
	return m_frequencyHz;
}

void b3SpringJoint::SetFrequency(float32 frequency)
{
	m_frequencyHz = frequency;
}

float32 b3SpringJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

void b3SpringJoint::SetDampingRatio(float32 ratio)
{
	m_dampingRatio = ratio;
}

void b3SpringJoint::InitializeConstraints(const b3SolverData* data) 
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

	m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

	// Singularity check.
	m_n = xB + m_rB - xA - m_rA;
	float32 length = b3Length(m_n);
	if (length > B3_LINEAR_SLOP)
	{
		m_n /= length;
	}
	else
	{
		m_n.SetZero();
	}
	
	// Compute the effective mass matrix
	b3Vec3 rnA = b3Cross(m_rA, m_n);
	b3Vec3 rnB = b3Cross(m_rB, m_n);

	float32 mass = m_mA + m_mB + b3Dot(m_iA * rnA, rnA) + b3Dot(m_iB * rnB, rnB);
	
	m_mass = mass > 0.0f ? 1.0f / mass : 0.0f;

	if (m_frequencyHz > 0.0f)
	{
		float32 C = length - m_length;
		
		// Angular frequency
		float32 omega = 2.0f * B3_PI * m_frequencyHz;

		// Damping coefficient
		float32 d = 2.0f * m_mass * m_dampingRatio * omega;

		// Spring stiffness
		float32 k = m_mass * omega * omega;

		// Box2D's Soft Constraints talk
		float32 h = data->dt;
		m_gamma = h * (d + h * k);
		m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
		m_bias = h * C * k * m_gamma;

		mass += m_gamma;
		m_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
	}
	else
	{
		m_bias = 0.0f;
		m_gamma = 0.0f;
	}
}

void b3SpringJoint::WarmStart(const b3SolverData* data) 
{
	b3Vec3 P = m_impulse * m_n;
	
	data->velocities[m_indexA].v -= m_mA * P;
	data->velocities[m_indexA].w -= m_iA * b3Cross(m_rA, P);
	data->velocities[m_indexB].v += m_mB * P;
	data->velocities[m_indexB].w += m_iB * b3Cross(m_rB, P);
}

void b3SpringJoint::SolveVelocityConstraints(const b3SolverData* data) 
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Vec3 dv = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
	float32 Cdot = b3Dot(m_n, dv);
	
	float32 impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
	m_impulse += impulse;
	
	b3Vec3 P = impulse * m_n;

	vA -= m_mA * P;
	wA -= m_iA * b3Cross(m_rA, P);
	
	vB += m_mB * P;
	wB += m_iB * b3Cross(m_rB, P);

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3SpringJoint::SolvePositionConstraints(const b3SolverData* data) 
{
	if (m_frequencyHz > 0.0f)
	{
		// There is no position correction for spring joints.
		B3_NOT_USED(data);
		return true;
	}

	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];
	float32 mA = m_mA;
	float32 mB = m_mB;

	b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);
	
	b3Vec3 n = xB + rB - xA - rA;
	float32 length = b3Length(n);
	float32 C = length - m_length;
	C = b3Clamp(C, -B3_MAX_LINEAR_CORRECTION, B3_MAX_LINEAR_CORRECTION);
	
	// Compute effective mass
	n.Normalize();
	
	b3Vec3 rnA = b3Cross(rA, n);
	b3Vec3 rnB = b3Cross(rB, n);
	float32 kMass = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);
	float32 mass = kMass > 0.0f ? 1.0f / kMass : 0.0f;
	float32 lambda = -mass * C;

	b3Vec3 impulse = lambda * n;
		
	xA -= mA * impulse;
	qA -= b3Derivative(qA, b3Mul(iA, b3Cross(rA, impulse)));
	iA = b3RotateToFrame(m_localInvIA, qA);

	xB += mB * impulse;
	qB += b3Derivative(qB, b3Mul(iB, b3Cross(rB, impulse)));
	iB = b3RotateToFrame(m_localInvIB, qB);

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return b3Abs(C) < B3_LINEAR_SLOP;
}

void b3SpringJoint::Draw() const 
{
	b3Vec3 a = GetBodyA()->GetWorldPoint(m_localAnchorA);
	b3Draw_draw->DrawPoint(a, 4.0f, b3Color_red);
	
	b3Vec3 b = GetBodyB()->GetWorldPoint(m_localAnchorB);
	b3Draw_draw->DrawPoint(b, 4.0f, b3Color_green);

	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}