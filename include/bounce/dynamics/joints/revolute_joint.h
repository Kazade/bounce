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

#include <bounce/dynamics/joints/joint.h>

struct b3RevoluteJointDef : public b3JointDef 
{
	b3RevoluteJointDef() 
	{
		type = e_revoluteJoint;
		localAnchorA.SetZero();
		localRotationA.SetIdentity();
		localAnchorB.SetZero();
		localRotationB.SetIdentity();
		referenceRotation.SetIdentity();
		enableLimit = false;
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		enableMotor = false;
		motorSpeed = 0.0f;
		maxMotorTorque = 0.0f;
	}

	// Initialize this definition from hinge axis, anchor point, and the lower and upper angle limits in radians.
	void Initialize(b3Body* bodyA, b3Body* bodyB, const b3Vec3& axis, const b3Vec3& anchor, float32 lowerAngle, float32 upperAngle);

	// The joint anchor relative body A's origin.
	b3Vec3 localAnchorA;

	// The joint orientation relative body A's orientation.
	b3Quat localRotationA;

	// The joint anchor relative body B's origin.
	b3Vec3 localAnchorB;

	// The joint orientation relative body B's orientation.
	b3Quat localRotationB;

	// The initial relative rotation from body A to body B.
	b3Quat referenceRotation;

	// Enable the joint limit.
	bool enableLimit;

	// The hinge lower angle limit in radians.
	float32 lowerAngle;
	
	// The hinge upper angle limit in radians.
	float32 upperAngle;

	// Enable the joint motor.
	bool enableMotor;

	// The desired motor speed in radians per second.
	float32 motorSpeed;
	
	// The maximum motor torque in Newton per meter.
	float32 maxMotorTorque;
};

// A revolute joint constrains two bodies to share a point and an axis while 
// they are free to rotate about the point and the axis. 
// The relative rotation about the shared axis is the joint rotation. 
// You can limit the relative rotation with a lower and upper angle limit. 
// You can use a motor to drive the relative rotation about the shared axis. 
// A maximum motor torque is provided so that infinite forces are not generated.
// This joint can be used to create structures such as doors.
class b3RevoluteJoint : public b3Joint
{
public:
	// Get the joint frame on body A in world coordinates.
	b3Transform GetFrameA() const;

	// Get the joint frame on body B in world coordinates.
	b3Transform GetFrameB() const;

	// Get the joint frame relative body A's frame.
	b3Transform GetLocalFrameA() const;

	// Get the joint frame relative body B's frame.
	b3Transform GetLocalFrameB() const;

	// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	// Set the joint limit enabled.
	void SetEnableLimit(bool bit);

	// Get the lower angle limit.
	float32 GetLowerLimit() const;

	// Get the upper angle limit.
	float32 GetUpperLimit() const;

	// Set the angle limits.
	void SetLimits(float32 lowerAngle, float32 upperAngle);

	// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	// Set the joint motor enabled.
	void SetEnableMotor(bool bit);

	// Get the desired motor speed in radians per second.
	float32 GetMotorSpeed() const;

	// Set the desired motor speed in radians per second.
	void SetMotorSpeed(float32 speed);

	// Get the maximum motor torque in Newton per meter.
	float32 GetMaxMotorTorque() const;

	// Set the maximum motor torque in Newton per meter.
	void SetMaxMotorTorque(float32 torque);

	// Draw this joint.
	void Draw() const;
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
	b3Quat m_referenceRotation;
	
	b3Vec3 m_localAnchorA;
	b3Quat m_localRotationA;

	b3Vec3 m_localAnchorB;
	b3Quat m_localRotationB;
	
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
	b3Mat33 m_localInvIA;
	b3Mat33 m_localInvIB;

	// Hinge motor
	b3Vec3 m_motor_J1; // 1x3 (row)
	b3Vec3 m_motor_J2; // 1x3 (row)
	float32 m_motorMass;
	float32 m_motorImpulse;

	// Hinge limit
	// The limit axis and constraint space mass are the same as the motor's
	b3LimitState m_limitState; // constraint state
	float32 m_limitImpulse;

	// Spherical
	b3Vec3 m_rA;
	b3Vec3 m_rB;
	b3Mat33 m_mass;
	b3Vec3 m_impulse;

	// Hinge
	b3Mat23 m_J1;
	b3Mat23 m_J2;
	b3Mat32 m_J1T;
	b3Mat32 m_J2T;
	b3Mat22 m_K;
	b3Vec2 m_axisImpulse;
};

#endif