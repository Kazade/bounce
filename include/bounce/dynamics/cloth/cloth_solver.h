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

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3Particle;
class b3Force;
class b3BodyContact;

struct b3DenseVec3;
struct b3DiagMat33;
struct b3SparseSymMat33;

struct b3ClothSolverDef
{
	b3StackAllocator* stack;
	u32 particleCapacity;
	u32 forceCapacity;
	u32 contactCapacity;
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

class b3ClothSolver
{
public:
	b3ClothSolver(const b3ClothSolverDef& def);
	~b3ClothSolver();
	
	void Add(b3Particle* p);
	void Add(b3Force* f);
	void Add(b3BodyContact* c);

	void Solve(float32 dt, const b3Vec3& gravity);
private:
	// Apply forces.
	void ApplyForces();
	
	// Apply constraints.
	void ApplyConstraints();

	// Compute A and b in Ax = b.
	void Compute_A_b(b3SparseSymMat33& A, b3DenseVec3& b) const;

	// Solve Ax = b.
	void Solve(b3DenseVec3& x, u32& iterations, const b3SparseSymMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const;

	b3StackAllocator* m_allocator;

	u32 m_particleCapacity;
	u32 m_particleCount;
	b3Particle** m_particles;

	u32 m_forceCapacity;
	u32 m_forceCount;
	b3Force** m_forces;

	u32 m_contactCapacity;
	u32 m_contactCount;
	b3BodyContact** m_contacts;

	u32 m_constraintCapacity;
	u32 m_constraintCount;
	b3AccelerationConstraint* m_constraints;

	b3ClothSolverData m_solverData;
};

#endif