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

#ifndef B3_MOUSE_JOINT_H
#define B3_MOUSE_JOINT_H

#include <bounce/dynamics/joints/joint.h>

// Mouse joint definition defines a world target 
// point and tunable parameters.
struct b3MouseJointDef : public b3JointDef 
{
	b3MouseJointDef() 
	{
		type = e_mouseJoint;
		target.SetZero();
		maxForce = 0.0f;
	}
	
	// The initial world target point. Initially is assumed 
	// to be coincident to the body anchor (satisfied constraint).
	b3Vec3 target;
	
	// Maximum joint reaction force in newtons.
	float32 maxForce; 
};

// A mouse joint is used to make a local point on a body 
// follow a defined world point.
class b3MouseJoint : public b3Joint 
{
public: 	
	// Get the world anchor point on body A.
	b3Vec3 GetAnchorA() const;

	// Get the world target point on body B.
	b3Vec3 GetAnchorB() const;

	// Get the world target point.
	const b3Vec3& GetTarget() const;
	
	// Set the world target point.
	void SetTarget(const b3Vec3& target);

	// Draw this joint.
	void Draw() const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3MouseJoint(const b3MouseJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Vec3 m_worldTargetA;
	b3Vec3 m_localAnchorB;
	float32 m_maxForce;

	// Solver temp
	u32 m_indexB;
	float32 m_mB;
	b3Mat33 m_iB;
	b3Vec3 m_localCenterB;
	b3Mat33 m_mass;
	b3Vec3 m_rB;
	b3Vec3 m_impulse;
	b3Vec3 m_C;
};

#endif