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

#include <bounce/softbody/joints/softbody_anchor.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody_solver.h>
#include <bounce/softbody/softbody_time_step.h>
#include <bounce/dynamics/body.h>

void b3SoftBodyAnchorDef::Initialize(b3Body* bA, b3SoftBodyNode* nB, const b3Vec3& anchor)
{
	bodyA = bA;
	nodeB = nB;
	
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = anchor - nodeB->GetPosition();
}

b3SoftBodyAnchor::b3SoftBodyAnchor(const b3SoftBodyAnchorDef& def)
{
	m_bodyA = def.bodyA;
	m_nodeB = def.nodeB;
	m_localAnchorA = def.localAnchorA;
	m_localAnchorB = def.localAnchorB;
	m_impulse.SetZero();
}

b3Vec3 b3SoftBodyAnchor::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3SoftBodyAnchor::GetAnchorB() const
{
	return m_localAnchorB + m_nodeB->GetPosition();
}

void b3SoftBodyAnchor::InitializeConstraints(const b3SoftBodySolverData* data)
{
	m_mA = m_bodyA->GetInverseMass();
	m_iA = m_bodyA->GetWorldInverseInertia();

	m_indexB = m_nodeB->m_meshIndex;
	m_mB = m_nodeB->m_invMass;
		
	b3Vec3 xA = m_bodyA->GetPosition();
	b3Quat qA = m_bodyA->GetOrientation();

	b3Vec3 xB = data->positions[m_indexB];

	b3Vec3 localCenterA = m_bodyA->GetSweep().localCenter;

	// Compute effective mass for the block solver
	m_rA = b3Mul(qA, m_localAnchorA - localCenterA);
	b3Vec3 rB = m_localAnchorB;

	b3Mat33 M = b3Diagonal(m_mA + m_mB);
	b3Mat33 RA = b3Skew(m_rA);
	b3Mat33 RAT = b3Transpose(RA);

	m_mass = M + RA * m_iA * RAT;

	b3Vec3 C = xB + rB - xA - m_rA;

	m_velocityBias = -data->step.inv_dt * B3_BAUMGARTE * C;
}

void b3SoftBodyAnchor::WarmStart(const b3SoftBodySolverData* data)
{
	b3Vec3 vA = m_bodyA->GetLinearVelocity();
	b3Vec3 wA = m_bodyA->GetAngularVelocity();
	
	b3Vec3 vB = data->velocities[m_indexB];

	vA -= m_mA * m_impulse;
	wA -= m_iA * b3Cross(m_rA, m_impulse);

	vB += m_mB * m_impulse;

	m_bodyA->SetLinearVelocity(vA);
	m_bodyA->SetAngularVelocity(wA);

	data->velocities[m_indexB] = vB;
}

void b3SoftBodyAnchor::SolveVelocityConstraints(const b3SoftBodySolverData* data)
{
	b3Vec3 vA = m_bodyA->GetLinearVelocity();
	b3Vec3 wA = m_bodyA->GetAngularVelocity();

	b3Vec3 vB = data->velocities[m_indexB];

	b3Vec3 Cdot = vB - vA - b3Cross(wA, m_rA);
	b3Vec3 impulse = m_mass.Solve(-Cdot + m_velocityBias);

	m_impulse += impulse;

	vA -= m_mA * impulse;
	wA -= m_iA * b3Cross(m_rA, impulse);

	vB += m_mB * impulse;

	m_bodyA->SetLinearVelocity(vA);
	m_bodyA->SetAngularVelocity(wA);

	data->velocities[m_indexB] = vB;
}

bool b3SoftBodyAnchor::SolvePositionConstraints(const b3SoftBodySolverData* data)
{
	B3_NOT_USED(data);
	return true;
}