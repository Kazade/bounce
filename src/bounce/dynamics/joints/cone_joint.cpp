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
	const b3Vec3& axis, const b3Vec3& anchor, scalar angle)
{
	bodyA = bA;
	bodyB = bB;

	b3Mat33 rotation;
	rotation.x = axis;
	b3ComputeBasis(rotation.x, rotation.y, rotation.z);

	b3Quat q = b3Mat33Quat(rotation);

	localAnchorA = bodyA->GetLocalPoint(anchor);
	localRotationA = bodyA->GetLocalFrame(q);

	localAnchorB = bodyB->GetLocalPoint(anchor);
	localRotationB = bodyB->GetLocalFrame(q);

	referenceRotation.SetIdentity();

	coneAngle = angle;
}

b3ConeJoint::b3ConeJoint(const b3ConeJointDef* def)
{
	m_type = e_coneJoint;
	m_localAnchorA = def->localAnchorA;
	m_localRotationA = def->localRotationA;
	m_localAnchorB = def->localAnchorB;
	m_localRotationB = def->localRotationB;
	m_referenceRotation = def->referenceRotation;
	
	m_enableConeLimit = def->enableConeLimit;
	m_coneAngle = def->coneAngle;

	m_enableTwistLimit = def->enableTwistLimit;
	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;

	m_coneState = e_inactiveLimit;
	m_coneImpulse = scalar(0);
	m_coneAxis.SetZero();
	
	m_twistState = e_inactiveLimit;
	m_twistImpulse = scalar(0);
	m_twistAxis.SetZero();
	
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

	b3Quat fA = qA * m_localRotationA;
	b3Quat fB = qB * m_localRotationB;

	b3Mat33 RfA = b3QuatMat33(fA);
	b3Mat33 RfB = b3QuatMat33(fB);

	// Linear constraint.
	{
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		// Compute effective mass matrix.
		b3Mat33 M = b3Diagonal(m_mA + m_mB);
		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		m_mass = M + RA * m_iA * RAT + RB * m_iB * RBT;
	}

	// Cone limit constraint.
	if (m_enableConeLimit)
	{
		b3Vec3 u1 = RfA.x;
		b3Vec3 u2 = RfB.x;

		m_coneAxis = b3Cross(u2, u1);

		scalar mass = b3Dot((m_iA + m_iB) * m_coneAxis, m_coneAxis);
		m_coneMass = mass > scalar(0) ? scalar(1) / mass : scalar(0);

		// C = cone / 2 - angle >= 0
		scalar cosine = b3Dot(u2, u1);
		scalar sine = b3Length(m_coneAxis);
		scalar angle = atan2(sine, cosine);
		if (scalar(0.5) * m_coneAngle < angle)
		{
			if (m_coneState != e_atLowerLimit)
			{
				m_coneState = e_atLowerLimit;
				m_coneImpulse = scalar(0);
			}
		}
		else
		{
			m_coneState = e_inactiveLimit;
			m_coneImpulse = scalar(0);
		}
	}
	else
	{
		m_coneState = e_inactiveLimit;
	}

	// Twist limit constraint
	if (m_enableTwistLimit)
	{
		// Jacobian
		m_twistAxis = RfA.x;

		// Compute effective mass
		m_twistMass = b3Dot((m_iA + m_iB) * m_twistAxis, m_twistAxis);
		if (m_twistMass > scalar(0))
		{
			m_twistMass = scalar(1) / m_twistMass;
		}

		// Joint rotation
		b3Quat q = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		// Joint angle
		scalar angle = scalar(2) * atan2(q.v.x, q.s);

		if (b3Abs(m_upperAngle - m_lowerAngle) < scalar(2) * B3_ANGULAR_SLOP)
		{
			if (m_twistState != e_equalLimits)
			{
				m_twistState = e_equalLimits;
				m_twistImpulse = scalar(0);
			}
		}
		else if (angle <= m_lowerAngle)
		{
			if (m_twistState != e_atLowerLimit)
			{
				m_twistState = e_atLowerLimit;
				m_twistImpulse = scalar(0);
			}
		}
		else if (angle >= m_upperAngle)
		{
			if (m_twistState != e_atUpperLimit)
			{
				m_twistState = e_atUpperLimit;
				m_twistImpulse = scalar(0);
			}
		}
		else
		{
			m_twistState = e_inactiveLimit;
			m_twistImpulse = scalar(0);
		}
	}
	else
	{
		m_twistState = e_inactiveLimit;
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

	if (m_enableConeLimit && m_coneState != e_inactiveLimit)
	{
		b3Vec3 L = m_coneImpulse * m_coneAxis;
		
		wA -= m_iA * L;
		wB += m_iB * L;
	}

	if (m_enableTwistLimit && m_twistState != e_inactiveLimit)
	{
		b3Vec3 L = m_twistImpulse * m_twistAxis;

		wA -= m_iA * L;
		wB += m_iB * L;
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

	// Solve linear constraint.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 P = m_mass.Solve(-Cdot);

		m_impulse += P;

		vA -= m_mA * P;
		wA -= m_iA * b3Cross(m_rA, P);

		vB += m_mB * P;
		wB += m_iB * b3Cross(m_rB, P);
	}

	// Solve cone constraint.
	if (m_enableConeLimit && m_coneState != e_inactiveLimit)
	{
		scalar Cdot = b3Dot(m_coneAxis, wB - wA);
		scalar impulse = -m_coneMass * Cdot;
		scalar oldImpulse = m_coneImpulse;
		m_coneImpulse = b3Max(m_coneImpulse + impulse, scalar(0));
		impulse = m_coneImpulse - oldImpulse;

		b3Vec3 P = impulse * m_coneAxis;

		wA -= m_iA * P;
		wB += m_iB * P;
	}

	// Solve twist constraint.
	if (m_enableTwistLimit && m_twistState != e_inactiveLimit)
	{
		scalar Cdot = b3Dot(wB - wA, m_twistAxis);
		scalar impulse = -m_twistMass * Cdot;

		if (m_twistState == e_equalLimits)
		{
			m_twistImpulse += impulse;
		}
		else if (m_twistState == e_atLowerLimit)
		{
			scalar oldImpulse = m_twistImpulse;
			m_twistImpulse = b3Max(m_twistImpulse + impulse, scalar(0));
			impulse = m_twistImpulse - oldImpulse;
		}
		else if (m_twistState == e_atUpperLimit)
		{
			scalar oldImpulse = m_twistImpulse;
			m_twistImpulse = b3Min(m_twistImpulse + impulse, scalar(0));
			impulse = m_twistImpulse - oldImpulse;
		}

		b3Vec3 P = impulse * m_twistAxis;

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

	scalar mA = m_mA;
	scalar mB = m_mB;
	
	// Solve limit constraints.
	scalar limitError = scalar(0);
	if (m_enableConeLimit || m_enableTwistLimit)
	{
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;

		b3Quat q1 = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		b3Quat q = q1;

		// Make sure the scalar part is positive.
		if (q.s < scalar(0))
		{
			// Negating still represent the same orientation.
			q = -q;
		}

		// Twist
		scalar x, xs;

		// Swing
		scalar y, z;
		
		scalar s = b3Sqrt(q.v.x * q.v.x + q.s * q.s);
		if (s < B3_EPSILON)
		{
			// Swing by 180 degrees is singularity.
			// Assume the twist is zero.
			x = scalar(0);
			xs = scalar(1);
			
			y = q.v.y;
			z = q.v.z;
		}
		else
		{
			scalar r = scalar(1) / s;

			x = q1.v.x * r;
			xs = q1.s * r;

			y = (q.s * q.v.y - q.v.x * q.v.z) * r;
			z = (q.s * q.v.z + q.v.x * q.v.y) * r;
		}

		// Solve twist
		if (m_enableTwistLimit)
		{
			// Joint angle
			scalar angle = scalar(2) * atan2(q1.v.x, q1.s);

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

				x = sin(theta);
				xs = cos(theta);
			}
		}

		// Twist
		b3Quat qt(x, scalar(0), scalar(0), xs);

		// Solve cone
		if (m_enableConeLimit)
		{
			// Half angle
			scalar angle = scalar(0.5) * m_coneAngle;

			// Circle radius
			scalar r = sin(scalar(0.5) * angle);

			// Circle clamp
			b3Vec2 p(y, z);
			if (b3LengthSquared(p) > r * r)
			{
				p.Normalize();
				
				// Allow some slop
				scalar rs = sin(scalar(0.5) * (angle + B3_ANGULAR_SLOP));

				p *= rs;

				y = p.x;
				z = p.y;
			}
		}

		// Swing
		b3Quat qs(scalar(0), y, z, b3Sqrt(b3Max(scalar(0), scalar(1) - y * y - z * z)));

		b3Quat q2 = qs * qt;

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

		limitError += b3Length(v);

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

	// Solve linear constraint.
	scalar linearError = scalar(0);
	{
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

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

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && limitError <= B3_ANGULAR_SLOP;
}

void b3ConeJoint::SetEnableConeLimit(bool bit)
{
	if (bit != m_enableConeLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_coneImpulse = scalar(0);
		m_coneState = e_inactiveLimit;
		m_enableConeLimit = bit;
	}
}

bool b3ConeJoint::IsConeLimitEnabled() const
{
	return m_enableConeLimit;
}

void b3ConeJoint::SetConeAngle(scalar angle)
{
	if (angle != m_coneAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_coneImpulse = scalar(0);
		m_coneAngle = angle;
	}
}

scalar b3ConeJoint::GetConeAngle() const
{
	return m_coneAngle;
}

void b3ConeJoint::SetEnableTwistLimit(bool bit)
{
	if (bit != m_enableTwistLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_twistImpulse = scalar(0);
		m_twistState = e_inactiveLimit;
		m_enableTwistLimit = bit;
	}
}

bool b3ConeJoint::IsTwistLimitEnabled() const
{
	return m_enableTwistLimit;
}

void b3ConeJoint::SetTwistLimits(scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);

	if (lower != m_lowerAngle || upper != m_upperAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_twistImpulse = scalar(0);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

scalar b3ConeJoint::GetTwistLowerAngle() const
{
	return m_lowerAngle;
}

scalar b3ConeJoint::GetTwistUpperAngle() const
{
	return m_upperAngle;
}

void b3ConeJoint::Draw() const
{
	b3Transform xfA(m_localAnchorA, m_localRotationA);
	xfA = GetBodyA()->GetWorldFrame(xfA);
	b3Draw_draw->DrawTransform(xfA);

	b3Transform xfB(m_localAnchorB, m_localRotationB);
	xfB = GetBodyB()->GetWorldFrame(xfB);
	b3Draw_draw->DrawTransform(xfB);
}