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

#ifndef B3_CONE_JOINT_H
#define B3_CONE_JOINT_H

#include <bounce/dynamics/joints/joint.h>

struct b3ConeJointDef : public b3JointDef
{
	b3ConeJointDef()
	{
		type = e_coneJoint;
		localFrameA.SetIdentity();
		localFrameB.SetIdentity();
		enableLimit = false;
		coneAngle = 0.0f;
	}

	// Initialize this definition from bodies, cone axis, anchor point, and full cone angle in radians.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& axis, const b3Vec3& anchor, float32 angle);

	// The joint frame relative to body A's frame.
	b3Transform localFrameA;

	// The joint frame relative to body B's frame.
	b3Transform localFrameB;

	// Enable cone angle limit.
	bool enableLimit;

	// The full cone angle in radians.
	float32 coneAngle;
};

// This joint constrains the bodies to share a common point (cone tip). 
// It also constrains the relative rotation about an axis perpendicular 
// to the cone axis.
// You can limit the relative rotation with a cone angle limit. 
// This joint can be used to create structures such as ragdolls.
class b3ConeJoint : public b3Joint
{
public:
	// Get the joint frame on body A in world coordinates.
	b3Transform GetFrameA() const;

	// Get the joint frame on body B in world coordinates.
	b3Transform GetFrameB() const;

	// Get the joint frame relative to body A's frame.
	const b3Transform& GetLocalFrameA() const;

	// Get the joint frame relative to body B's frame.
	const b3Transform& GetLocalFrameB() const;

	// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	// Set the joint limit enabled.
	void SetEnableLimit(bool bit);

	// Get the cone angle in radians.
	float32 GetConeAngle() const;

	// Set the cone angle in radians.
	void SetConeAngle(float32 angle);

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3Body;
	friend class b3World;
	friend class b3Island;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3ConeJoint(const b3ConeJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Transform m_localFrameA;
	b3Transform m_localFrameB;
	float32 m_coneAngle;
	bool m_enableLimit;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	float32 m_mA;
	float32 m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;
	
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	
	// Point-to-point
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_mass;
	b3Vec3 m_impulse;

	// Limit
	b3Vec3 m_limitAxis;
	float32 m_limitMass;
	float32 m_limitImpulse;
	b3LimitState m_limitState;
};

#endif