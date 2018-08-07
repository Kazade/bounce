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

#ifndef B3_CONTACT_SOLVER_H
#define B3_CONTACT_SOLVER_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/mat22.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/contacts/manifold.h>

class b3StackAllocator;
class b3Contact;
struct b3Position;
struct b3Velocity;

struct b3PositionConstraintPoint
{
	b3Vec3 localNormalA;
	b3Vec3 localPointA;
	b3Vec3 localPointB;
};

struct b3PositionConstraintManifold
{
	b3PositionConstraintPoint* points;
	u32 pointCount;
};

struct b3ContactPositionConstraint 
{
	u32 indexA;
	float32 invMassA;
	b3Mat33 localInvIA;
	float32 radiusA;
	b3Vec3 localCenterA;
	u32 indexB;
	b3Vec3 localCenterB;
	float32 invMassB;
	b3Mat33 localInvIB;
	float32 radiusB;
	b3PositionConstraintManifold* manifolds;
	u32 manifoldCount;
};

struct b3VelocityConstraintPoint 
{
	b3Vec3 rA;
	b3Vec3 rB;

	b3Vec3 normal;
	float32 normalMass;
	float32 normalImpulse;
	float32 velocityBias;
};

struct b3VelocityConstraintManifold
{
	b3Vec3 rA;
	b3Vec3 rB;
	
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
	//float32 leverArm;

	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;
	float32 motorImpulse;
	float32 motorMass;
	
	b3VelocityConstraintPoint* points;
	u32 pointCount;
};

// The idea is to allow anything to bounce off an inelastic surface.
inline float32 b3MixRestitution(float32 e1, float32 e2)
{
	return b3Max(e1, e2);
}

// The idea is to drive the restitution to zero. 
inline float32 b3MixFriction(float32 u1, float32 u2)
{
	return b3Sqrt(u1 * u2);
}

struct b3ContactVelocityConstraint 
{
	u32 indexA;
	float32 invMassA;
	b3Mat33 invIA;
	float32 invMassB;
	u32 indexB;
	b3Mat33 invIB;
	float32 friction;
	float32 restitution;
	b3VelocityConstraintManifold* manifolds;
	u32 manifoldCount;
};

struct b3ContactSolverDef 
{
	b3Position* positions;
	b3Velocity* velocities;
	b3Mat33* invInertias;
	b3Contact** contacts;
	u32 count;
	b3StackAllocator* allocator;
	float32 dt;
};

class b3ContactSolver 
{
public:
	b3ContactSolver(const b3ContactSolverDef* def);
	~b3ContactSolver();

	void InitializeConstraints();
	void WarmStart();
	
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
protected:
	b3Position* m_positions;
	b3Velocity* m_velocities;
	b3Mat33* m_inertias;
	b3Contact** m_contacts;
	b3ContactPositionConstraint* m_positionConstraints;
	b3ContactVelocityConstraint* m_velocityConstraints;
	u32 m_count;
	float32 m_dt, m_invDt;
	b3StackAllocator* m_allocator;
};

#endif
