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

#ifndef B3_SPHERE_JOINT_H
#define B3_SPHERE_JOINT_H

#include <bounce\dynamics\joints\joint.h>

struct b3SphereJointDef : public b3JointDef
{
	b3SphereJointDef()
	{
		type = b3JointType::e_sphereJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
	}

	// Initialize this definition.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor);

	b3Vec3 localAnchorA;
	b3Vec3 localAnchorB;
};

// A ball-in-socket joint.
class b3SphereJoint : public b3Joint
{
public:
	// Get the local anchor point on body A.
	const b3Vec3& GetLocalAnchorA() const;
	
	// Set the local anchor point on body A.
	void SetLocalAnchorA(const b3Vec3& point);

	// Get the local anchor point on body B.
	const b3Vec3& GetLocalAnchorB() const;
	
	// Set the local anchor point on body B.
	void SetLocalAnchorB(const b3Vec3& point);

	// Implement b3Joint
	void Draw(b3Draw* b3Draw) const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3SphereJoint(const b3SphereJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// The local joint frames on each body.
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;

	// Temporary data copied from the joint solver
	// to reduce cache misses.
	u32 m_indexA;
	u32 m_indexB;
	float32 m_mA;
	float32 m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;

	// Constraint data.
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_mass;
	b3Vec3 m_impulse;
};

inline const b3Vec3& b3SphereJoint::GetLocalAnchorA() const
{
	return m_localAnchorA;
}

inline void b3SphereJoint::SetLocalAnchorA(const b3Vec3& point)
{
	m_localAnchorA = point;
}

inline const b3Vec3& b3SphereJoint::GetLocalAnchorB() const
{
	return m_localAnchorB;
}

inline void b3SphereJoint::SetLocalAnchorB(const b3Vec3& point)
{
	m_localAnchorB = point;
}

#endif
