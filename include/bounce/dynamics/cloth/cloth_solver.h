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

#ifndef B3_CLOTH_SOLVER_H
#define B3_CLOTH_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3Particle;
class b3Body;
class b3Force;
class b3BodyContact;
class b3ParticleContact;
class b3TriangleContact;

struct b3DenseVec3;
struct b3DiagMat33;
struct b3SparseSymMat33;

struct b3ClothSolverDef
{
	b3StackAllocator* stack;
	u32 particleCapacity;
	u32 forceCapacity;
	u32 bodyContactCapacity;
	u32 particleContactCapacity;
	u32 triangleContactCapacity;
};

struct b3ClothSolverData
{
	b3DenseVec3* x;
	b3DenseVec3* v;
	b3DenseVec3* f;
	b3DenseVec3* y;
	b3SparseSymMat33* dfdx;
	b3SparseSymMat33* dfdv;
	b3DiagMat33* S;
	b3DenseVec3* z;
	float32 dt;
	float32 invdt;
};

struct b3AccelerationConstraint
{
	u32 i1;
	u32 ndof;
	b3Vec3 p, q, z;

	void Apply(const b3ClothSolverData* data);
};

struct b3ClothSolverContactVelocityConstraint
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

struct b3ClothSolverContactPositionConstraint
{
	u32 indexA;
	float32 invMassA;
	b3Mat33 invIA;
	float32 radiusA;
	b3Vec3 localCenterA;
	
	b3Body* bodyB;
	b3Vec3 localCenterB;
	float32 invMassB;
	b3Mat33 invIB;
	float32 radiusB;

	b3Vec3 rA;
	b3Vec3 rB;

	b3Vec3 localNormalA;
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

class b3ClothSolver
{
public:
	b3ClothSolver(const b3ClothSolverDef& def);
	~b3ClothSolver();
	
	void Add(b3Particle* p);
	void Add(b3Force* f);
	void Add(b3BodyContact* c);
	void Add(b3ParticleContact* c);
	void Add(b3TriangleContact* c);

	void Solve(float32 dt, const b3Vec3& gravity);
private:
	// Apply forces.
	void ApplyForces();
	
	// Apply constraints.
	void ApplyConstraints();

	// Solve Ax = b.
	void Solve(b3DenseVec3& x, u32& iterations, const b3SparseSymMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const;

	// 
	void InitializeBodyContactConstraints();
	
	// 
	void InitializeParticleContactConstraints();
	
	// 
	void InitializeTriangleContactConstraints();
	
	//
	void WarmStart();

	// 
	void SolveBodyContactVelocityConstraints();

	// 
	void SolveParticleContactVelocityConstraints();
	
	// 
	void SolveTriangleContactVelocityConstraints();
	
	// 
	void StoreImpulses();

	// 
	bool SolveBodyContactPositionConstraints();
	
	// 
	bool SolveParticleContactPositionConstraints();
	
	// 
	bool SolveTriangleContactPositionConstraints();
	
	b3StackAllocator* m_allocator;

	u32 m_particleCapacity;
	u32 m_particleCount;
	b3Particle** m_particles;

	u32 m_forceCapacity;
	u32 m_forceCount;
	b3Force** m_forces;
	
	u32 m_constraintCapacity;
	u32 m_constraintCount;
	b3AccelerationConstraint* m_constraints;

	u32 m_bodyContactCapacity;
	u32 m_bodyContactCount;
	b3BodyContact** m_bodyContacts;
	b3ClothSolverContactVelocityConstraint* m_bodyVelocityConstraints;
	b3ClothSolverContactPositionConstraint* m_bodyPositionConstraints;

	u32 m_particleContactCapacity;
	u32 m_particleContactCount;
	b3ParticleContact** m_particleContacts;
	b3ClothSolverParticleContactVelocityConstraint* m_particleVelocityConstraints;
	b3ClothSolverParticleContactPositionConstraint* m_particlePositionConstraints;

	u32 m_triangleContactCapacity;
	u32 m_triangleContactCount;
	b3TriangleContact** m_triangleContacts;
	b3ClothSolverTriangleContactVelocityConstraint* m_triangleVelocityConstraints;
	b3ClothSolverTriangleContactPositionConstraint* m_trianglePositionConstraints;
	
	b3ClothSolverData m_solverData;
};

#endif