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

#include <bounce\dynamics\joints\joint.h>

struct b3MouseJointDef : public b3JointDef 
{
	b3MouseJointDef() 
	{
		type = e_mouseJoint;
		worldAnchorA.SetZero();
		localAnchorB.SetZero();
		maxForce = 0.0f;
	}
	
	b3Vec3 worldAnchorA;
	b3Vec3 localAnchorB;
	float32 maxForce; 
};

class b3MouseJoint : public b3Joint 
{
public : 	
	// Get the world space anchor point on the first body (usually the mouse world space position).
	b3Vec3 GetWorldAnchorA() const;

	// Set the world space anchor position on the first body.
	void SetWorldAnchorA(const b3Vec3& v);

	// Get the world space anchor point on the first body (usually the mouse world space position).
	b3Vec3 GetWorldAnchorB() const;

	// Get the local space anchor point on the second body (usually the ray cast intersection).
	const b3Vec3& GetLocalAnchorB() const;

	// Set the mouse position on the space of the second body (usually the ray cast intersection).
	void SetLocalAnchorB(const b3Vec3& v);

	// Implement b3Joint.
	void Draw(b3Draw* b3Draw) const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3MouseJoint(const b3MouseJointDef* def);

	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// The two anchor points on each body.
	// The first body has infinite mass. Therefore,
	// we store the world space anchor point.
	b3Vec3 m_worldAnchorA;
	b3Vec3 m_localAnchorB;
	float32 m_maxForce; // maximum reaction force in Newtons

	// Constraint data for the solver.
	u32 m_indexB;
	float32 m_mB;
	b3Mat33 m_iB;
	b3Mat33 m_mass;
	b3Vec3 m_rB;
	b3Vec3 m_impulse;
	b3Vec3 m_C;
};

inline b3Vec3 b3MouseJoint::GetWorldAnchorA() const 
{
	return m_worldAnchorA;
}

inline void b3MouseJoint::SetWorldAnchorA(const b3Vec3& v) 
{
	m_worldAnchorA = v;
}

inline const b3Vec3& b3MouseJoint::GetLocalAnchorB() const 
{
	return m_localAnchorB;
}

inline void b3MouseJoint::SetLocalAnchorB(const b3Vec3& v) 
{
	m_localAnchorB = v;
}

#endif
