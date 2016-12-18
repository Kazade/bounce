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

#ifndef B3_REVOLUTE_JOINT_H
#define B3_REVOLUTE_JOINT_H

#include <bounce\dynamics\joints\joint.h>
#include <bounce\common\math\mat.h>

struct b3RevoluteJointDef : public b3JointDef 
{
	b3RevoluteJointDef() 
	{
		type = e_revoluteJoint;
		localFrameA.SetIdentity();
		localFrameB.SetIdentity();
		enableLimit = false;
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		enableMotor = false;
		motorSpeed = 0.0f;
		maxMotorTorque = 0.0f;
	}

	// Initialize this definition given an axis, anchor point, and the lower and upper angle limits in radians.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& axis, const b3Vec3& anchor, float32 lowerAngle, float32 upperAngle);

	// The joint frame relative to the frame of body A.
	b3Transform localFrameA;
	
	// The joint frame relative to the frame of body B.
	b3Transform localFrameB;

	// Enable the joint limit.
	bool enableLimit;

	// The lower angle limit in radians.
	float32 lowerAngle;
	
	// The upper angle limit in radians.
	float32 upperAngle;

	// Enable the joint motor.
	bool enableMotor;

	// The desired motor speed in radians per second.
	float32 motorSpeed;
	
	// The maximum motor torque in Newton per meter.
	float32 maxMotorTorque;
};

// A revolute joint constrains two bodies to share a common point while they
// are free to rotate about the point and a given axis. 
// The relative rotation about the shared axis
// is the joint angle. You can limit the relative rotation with
// a lower and upper angle limit. Also, you can use a motor
// to drive the relative rotation about the shared axis. 
// A maximum motor torque is provided so that infinite forces are not generated.
class b3RevoluteJoint : public b3Joint
{
public:
	// Get the joint frame relative to the frame of body A.
	const b3Transform& GetFrameA() const;	

	// Set the joint frame relative to the frame of body A.
	void SetFrameA(const b3Transform& xf);

	// Get the joint frame relative to the frame of body B.
	const b3Transform& GetFrameB() const;
	
	// Set the joint frame relative to the frame of body B.
	void SetFrameB(const b3Transform& xf);

	// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	// Set the joint limit enabled.
	void SetEnableLimit(bool bit);

	// Get the lower angle limit.
	float32 GetLowerLimit() const;

	// Get the upper limit.
	float32 GetUpperLimit() const;

	// Set the angle limits.
	void SetLimits(float32 lowerAngle, float32 upperAngle);

	// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	// Set the joint motor enabled.
	void SetEnableMotor(bool bit);

	// Get the desired motor speed (radians per second).
	float32 GetMotorSpeed() const;

	// Set the desired motor speed (radians per second).
	void SetMotorSpeed(float32 speed);

	// Get the maximum motor torque (Newton per meter).
	float32 GetMaxMotorTorque() const;

	// Set the maximum motor torque (Newton per meter).
	void SetMaxMotorTorque(float32 torque);

	// Draw this joint.
	void Draw(b3Draw* b3Draw) const;
private:
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;

	b3RevoluteJoint(const b3RevoluteJointDef* def);
	
	virtual void InitializeConstraints(const b3SolverData* data);
	virtual void WarmStart(const b3SolverData* data);
	virtual void SolveVelocityConstraints(const b3SolverData* data);
	virtual bool SolvePositionConstraints(const b3SolverData* data);

	// Solver shared
	b3Transform m_localFrameA;
	b3Transform m_localFrameB;
	
	bool m_enableMotor;
	float32 m_motorSpeed;
	float32 m_maxMotorTorque;
	
	bool m_enableLimit;
	float32 m_lowerAngle;
	float32 m_upperAngle;

	// Solver temp
	u32 m_indexA;
	u32 m_indexB;
	float32 m_mA;
	float32 m_mB;
	b3Mat33 m_iA;
	b3Mat33 m_iB;
	b3Vec3 m_localCenterA;
	b3Vec3 m_localCenterB;
	
	// Motor
	// The limit axis is the same as the motor axis
	float32 m_motorMass;
	float32 m_motorImpulse;

	// Limit
	b3Vec3 m_limitAxis; // axis of rotation for limit contraint
	float32 m_limitImpulse;
	b3LimitState m_limitState; // constraint state

	// Point-to-point + axes-to-axes
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Vec3 m_nA;
	b3Vec3 m_nB;
	b3Mat<5, 5> m_mass;
	b3Vec<5> m_impulse;
};

#endif
