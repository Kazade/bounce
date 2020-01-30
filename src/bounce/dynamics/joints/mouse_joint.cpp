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

#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

b3MouseJoint::b3MouseJoint(const b3MouseJointDef* def) 
{
	m_type = e_mouseJoint;
	m_worldTargetA = def->target;
	m_localAnchorB = def->bodyB->GetLocalPoint(def->target);
	m_maxForce = def->maxForce;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;
	m_impulse.SetZero();
}

static B3_FORCE_INLINE void b3ComputeSoftConstraintCoefficients(scalar& gamma, scalar& bias,
	scalar frequencyHz, scalar dampingRatio, scalar m, scalar C, scalar h)
{
	// Frequency
	scalar omega = scalar(2) * B3_PI * frequencyHz;

	// Spring stiffness
	scalar k = omega * omega * m;

	// Damping coefficient
	scalar d = scalar(2) * dampingRatio * omega * m;

	// Magic formulas
	gamma = h * (d + h * k);
	gamma = gamma != scalar(0) ? scalar(1) / gamma : scalar(0);
	bias = gamma * h * k * C;
}

void b3MouseJoint::InitializeConstraints(const b3SolverData* data) 
{
	b3Body* m_bodyB = GetBodyB();

	m_indexB = m_bodyB->m_islandID;
	m_mB = m_bodyB->m_invMass;
	m_iB = data->invInertias[m_indexB];
	m_localCenterB = m_bodyB->m_sweep.localCenter;

	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;

	// Compute the effective mass matrix.
	m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);
	b3Mat33 M = b3Diagonal(m_mB);
	b3Mat33 RB = b3Skew(m_rB);
	b3Mat33 RBT = b3Transpose(RB);
	b3Mat33 invM = M + RB * m_iB * RBT;
	b3Mat33 m = b3Inverse(invM);

	b3Vec3 C = xB + m_rB - m_worldTargetA;

	scalar h = data->dt;

	b3Vec3 gamma, bias;

	b3ComputeSoftConstraintCoefficients(gamma.x, bias.x, m_frequencyHz, m_dampingRatio, m.x.x, C.x, h);
	b3ComputeSoftConstraintCoefficients(gamma.y, bias.y, m_frequencyHz, m_dampingRatio, m.y.y, C.y, h);
	b3ComputeSoftConstraintCoefficients(gamma.z, bias.z, m_frequencyHz, m_dampingRatio, m.z.z, C.z, h);

	m_gamma = b3Diagonal(gamma.x, gamma.y, gamma.z);
	m_bias = bias;

	invM += m_gamma;
	m_mass = b3Inverse(invM);
}

void b3MouseJoint::WarmStart(const b3SolverData* data) 
{
	data->velocities[m_indexB].v += m_mB * m_impulse;
	data->velocities[m_indexB].w += m_iB * b3Cross(m_rB, m_impulse);
}

void b3MouseJoint::SolveVelocityConstraints(const b3SolverData* data) 
{
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Vec3 Cdot = vB + b3Cross(wB, m_rB);

	b3Vec3 impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
	b3Vec3 oldImpulse = m_impulse;
	m_impulse += impulse;
	scalar maxImpulse = data->dt * m_maxForce;
	if (b3Dot(m_impulse, m_impulse) > maxImpulse * maxImpulse)
	{
		m_impulse *= maxImpulse / b3Length(m_impulse);
	}	
	impulse = m_impulse - oldImpulse;

	vB += m_mB * impulse;
	wB += m_iB * b3Cross(m_rB, impulse);
	
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3MouseJoint::SolvePositionConstraints(const b3SolverData* data) 
{
    B3_NOT_USED(data);
	// There is no position correction for this constraint.
	return true;
}

b3Vec3 b3MouseJoint::GetAnchorA() const 
{
	return m_worldTargetA;
}

b3Vec3 b3MouseJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

const b3Vec3& b3MouseJoint::GetTarget() const
{
	return m_worldTargetA;
}

void b3MouseJoint::SetTarget(const b3Vec3& target)
{
	m_worldTargetA = target;
	GetBodyB()->SetAwake(true);
}

void b3MouseJoint::Draw() const 
{
	b3Vec3 a = GetAnchorA();
	b3Vec3 b = GetAnchorB();

	b3Draw_draw->DrawPoint(a, scalar(4), b3Color_green);
	b3Draw_draw->DrawPoint(b, scalar(4), b3Color_red);
	b3Draw_draw->DrawSegment(a, b, b3Color_yellow);
}