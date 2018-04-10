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

#ifndef B3_WELD_JOINT_H
#define B3_WELD_JOINT_H

#include <bounce/dynamics/joints/joint.h>

struct b3WeldJointDef : public b3JointDef
{
	b3WeldJointDef()
	{
		type = e_weldJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		referenceRotation.SetIdentity();
	}

	// Initialize this definition from bodies and world anchor point.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& anchor);

	// The joint anchor relative body A's origin.
	b3Vec3 localAnchorA;

	// The joint anchor relative body B's origin.
	b3Vec3 localAnchorB;

	// The initial relative rotation from body A to body B.
	b3Quat referenceRotation;
};

// A weld joint removes the relative movement between two bodies. 
// You need to specify the relative rotation and the local anchor points. 
// @todo Soft this constraint.
class b3WeldJoint : public b3Joint
{
public:
	// Get the anchor point on body A in world coordinates.
	b3Vec3 GetAnchorA() const;

	// Get the anchor point on body B in world coordinates.
	b3Vec3 GetAnchorB() const;

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3WeldJoint(const b3WeldJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;
	b3Quat m_referenceRotation;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	float32 m_mA;
	float32 m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;

	// Point-to-point
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Vec3 m_impulse;
	b3Mat33 m_mass;

	// Weld constraint
	b3Mat33 m_J1;
	b3Mat33 m_J2;
	b3Mat33 m_J1T;
	b3Mat33 m_J2T;
	b3Vec3 m_axisImpulse;
	b3Mat33 m_K;
};

#endif