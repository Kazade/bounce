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

struct b3DenseVec3;
struct b3DiagMat33;
struct b3SparseSymMat33;
struct b3SymMat33;

class b3Particle;
class b3Force;
struct b3BodyContact;

struct b3ClothSolverDef
{
	b3StackAllocator* stack;
	u32 particleCapacity;
	u32 forceCapacity;
	u32 contactCapacity;
};

struct b3ClothSolverData
{
	b3Vec3* x;
	b3Vec3* v;
	b3Vec3* f;
	b3SymMat33* dfdx;
	b3SymMat33* dfdv;
	float32 dt;
	float32 invdt;
};

struct b3AccelerationConstraint
{
	u32 i1;
	u32 ndof;
	b3Vec3 p, q, z;
};

struct b3SymMat33
{
	b3SymMat33(b3StackAllocator* a, u32 m, u32 n)
	{
		allocator = a;
		M = m;
		N = n;
		values = (b3Mat33*)b3Alloc(M * N * sizeof(b3Mat33));
	}

	~b3SymMat33()
	{
		b3Free(values);
	}

	b3Mat33& operator()(u32 i, u32 j)
	{
		return values[i * N + j];
	}

	const b3Mat33& operator()(u32 i, u32 j) const
	{
		return values[i * N + j];
	}

	void SetZero()
	{
		for (u32 v = 0; v < M * N; ++v)
		{
			values[v].SetZero();
		}
	}

	u32 M;
	u32 N;
	b3Mat33* values;
	b3StackAllocator* allocator;
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
	// Initialize forces.
	void InitializeForces();
	
	// Apply forces.
	void ApplyForces();
	
	// Initialize constraints.
	void InitializeConstraints();

	// Compute A and b in Ax = b
	void Compute_A_b(b3SparseSymMat33& A, b3DenseVec3& b, const b3DenseVec3& f, const b3DenseVec3& x, const b3DenseVec3& v, const b3DenseVec3& y) const;

	// Compute S and z.
	void Compute_S_z(b3DiagMat33& S, b3DenseVec3& z);

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