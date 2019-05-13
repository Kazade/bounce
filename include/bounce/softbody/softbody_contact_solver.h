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

#ifndef B3_SOFT_BODY_CONTACT_SOLVER_H
#define B3_SOFT_BODY_CONTACT_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3SoftBodyNode;
class b3Body;

class b3NodeBodyContact;

struct b3DenseVec3;

struct b3SoftBodySolverBodyContactVelocityConstraint
{
	u32 indexA;
	float32 invMassA;
	b3Mat33 invIA;

	b3Body* bodyB;
	float32 invMassB;
	b3Mat33 invIB;

	float32 friction;

	b3Vec3 point;
	b3Vec3 rA;
	b3Vec3 rB;

	b3Vec3 normal;
	float32 normalMass;
	float32 normalImpulse;
	float32 velocityBias;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;
};

struct b3SoftBodySolverBodyContactPositionConstraint
{
	u32 indexA;
	float32 invMassA;
	b3Mat33 invIA;
	float32 radiusA;
	b3Vec3 localCenterA;

	b3Body* bodyB;
	float32 invMassB;
	b3Mat33 invIB;
	float32 radiusB;
	b3Vec3 localCenterB;

	b3Vec3 rA;
	b3Vec3 rB;

	b3Vec3 normalA;
	b3Vec3 localPointA;
	b3Vec3 localPointB;
};

struct b3SoftBodyContactSolverDef
{
	b3StackAllocator* allocator;

	b3DenseVec3* positions;
	b3DenseVec3* velocities;

	u32 bodyContactCapacity;
};

inline float32 b3MixFriction(float32 u1, float32 u2)
{
	return b3Sqrt(u1 * u2);
}

class b3SoftBodyContactSolver
{
public:
	b3SoftBodyContactSolver(const b3SoftBodyContactSolverDef& def);
	~b3SoftBodyContactSolver();

	void Add(b3NodeBodyContact* c);

	void InitializeBodyContactConstraints();

	void WarmStart();

	void SolveBodyContactVelocityConstraints();

	void StoreImpulses();

	bool SolveBodyContactPositionConstraints();
protected:
	b3StackAllocator* m_allocator;

	b3DenseVec3* m_positions;
	b3DenseVec3* m_velocities;

	u32 m_bodyContactCapacity;
	u32 m_bodyContactCount;
	b3NodeBodyContact** m_bodyContacts;
	
	b3SoftBodySolverBodyContactVelocityConstraint* m_bodyVelocityConstraints;
	b3SoftBodySolverBodyContactPositionConstraint* m_bodyPositionConstraints;
};

#endif