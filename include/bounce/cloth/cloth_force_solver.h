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

#ifndef B3_CLOTH_FORCE_SOLVER_H
#define B3_CLOTH_FORCE_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3Particle;
class b3Force;

struct b3DenseVec3;
struct b3DiagMat33;
struct b3SparseMat33;
struct b3SparseMat33View;

struct b3ClothForceSolverDef
{
	b3StackAllocator* stack;
	u32 particleCount;
	b3Particle** particles;
	u32 forceCount;
	b3Force** forces;
};

struct b3ClothForceSolverData
{
	b3DenseVec3* x;
	b3DenseVec3* v;
	b3DenseVec3* f;
	b3DenseVec3* y;
	b3SparseMat33* dfdx;
	b3SparseMat33* dfdv;
	b3DiagMat33* S;
	b3DenseVec3* z;
};

class b3ClothForceSolver
{
public:
	b3ClothForceSolver(const b3ClothForceSolverDef& def);
	~b3ClothForceSolver();

	void Solve(float32 dt, const b3Vec3& gravity);
private:
	void ApplyForces();

	b3StackAllocator* m_allocator;

	u32 m_particleCount;
	b3Particle** m_particles;

	u32 m_forceCount;
	b3Force** m_forces;

	b3ClothForceSolverData m_solverData;
};

#endif