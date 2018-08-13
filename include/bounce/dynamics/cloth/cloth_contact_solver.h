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

#ifndef B3_CLOTH_CONTACT_SOLVER_H
#define B3_CLOTH_CONTACT_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3Particle;
class b3Body;

class b3BodyContact;
class b3ParticleContact;
class b3TriangleContact;

struct b3DenseVec3;

struct b3ClothSolverBodyContactVelocityConstraint
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

struct b3ClothSolverBodyContactPositionConstraint
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

	b3Vec3 localPointA;
	b3Vec3 localPointB;
};

struct b3ClothSolverParticleContactVelocityConstraint
{
	u32 indexA;
	float32 invMassA;

	u32 indexB;
	float32 invMassB;

	float32 friction;

	b3Vec3 point;

	b3Vec3 normal;
	float32 normalMass;
	float32 normalImpulse;
	float32 velocityBias;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;
};

struct b3ClothSolverParticleContactPositionConstraint
{
	u32 indexA;
	float32 invMassA;
	float32 radiusA;

	u32 indexB;
	float32 invMassB;
	float32 radiusB;
};

struct b3ClothSolverTriangleContactVelocityConstraint
{
	u32 indexA;
	float32 invMassA;

	u32 indexB;
	float32 invMassB;
	u32 indexC;
	float32 invMassC;
	u32 indexD;
	float32 invMassD;

	b3Vec3 JA;
	b3Vec3 JB;
	b3Vec3 JC;
	b3Vec3 JD;

	float32 normalMass;
	float32 normalImpulse;
};

struct b3ClothSolverTriangleContactPositionConstraint
{
	u32 indexA;
	float32 invMassA;
	float32 radiusA;

	u32 indexB;
	float32 invMassB;
	u32 indexC;
	float32 invMassC;
	u32 indexD;
	float32 invMassD;
	float32 triangleRadius;

	bool front;
};

struct b3ClothContactSolverDef
{
	b3StackAllocator* allocator;
	
	b3DenseVec3* positions;
	b3DenseVec3* velocities;
	
	u32 bodyContactCount;
	b3BodyContact** bodyContacts;
	
	u32 particleContactCount;
	b3ParticleContact** particleContacts;

	u32 triangleContactCount;
	b3TriangleContact** triangleContacts;
};

inline float32 b3MixFriction(float32 u1, float32 u2)
{
	return b3Sqrt(u1 * u2);
}

class b3ClothContactSolver
{
public:
	b3ClothContactSolver(const b3ClothContactSolverDef& def);
	~b3ClothContactSolver();

	void InitializeBodyContactConstraints();

	void InitializeParticleContactConstraints();

	void InitializeTriangleContactConstraints();

	void WarmStart();

	void SolveBodyContactVelocityConstraints();

	void SolveParticleContactVelocityConstraints();

	void SolveTriangleContactVelocityConstraints();

	void StoreImpulses();

	bool SolveBodyContactPositionConstraints();

	bool SolveParticleContactPositionConstraints();

	bool SolveTriangleContactPositionConstraints();

protected:
	b3StackAllocator* m_allocator;

	b3DenseVec3* m_positions;
	b3DenseVec3* m_velocities;

	u32 m_bodyContactCount;
	b3BodyContact** m_bodyContacts;
	b3ClothSolverBodyContactVelocityConstraint* m_bodyVelocityConstraints;
	b3ClothSolverBodyContactPositionConstraint* m_bodyPositionConstraints;

	u32 m_particleContactCount;
	b3ParticleContact** m_particleContacts;
	b3ClothSolverParticleContactVelocityConstraint* m_particleVelocityConstraints;
	b3ClothSolverParticleContactPositionConstraint* m_particlePositionConstraints;

	u32 m_triangleContactCount;
	b3TriangleContact** m_triangleContacts;
	b3ClothSolverTriangleContactVelocityConstraint* m_triangleVelocityConstraints;
	b3ClothSolverTriangleContactPositionConstraint* m_trianglePositionConstraints;
};

#endif