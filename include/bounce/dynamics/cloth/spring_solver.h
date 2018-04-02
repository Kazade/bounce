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

#ifndef B3_SPRING_SOLVER_H
#define B3_SPRING_SOLVER_H

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>

class b3SpringCloth;
class b3StackAllocator;

struct b3DenseVec3;
struct b3SparseMat33;

struct b3MassContact;
struct b3Spring;

enum b3MassType;

struct b3SpringSolverDef
{
	b3SpringCloth* cloth;
	float32 dt;
};

class b3SpringSolver
{
public:
	b3SpringSolver(const b3SpringSolverDef& def);

	~b3SpringSolver();

	void Solve(b3DenseVec3& extraForces);

	u32 GetIterations() const;
private:
	// Apply internal forces and store their unique derivatives.
	void ApplySpringForces();

	// Compute A and b in Ax = b
	void Compute_A_b(b3SparseMat33& A, b3DenseVec3& b) const;

	// Solve Ax = b using the Modified Conjugate Gradient (MCG). 
	// Output x and the residual error f.
	void Solve_MCG(b3DenseVec3& x, const b3SparseMat33& A, b3DenseVec3& f, u32& iterations, const b3DenseVec3& b) const;

	// Solve Ax = b using MCG with Jacobi preconditioning.
	// Output x and the residual error f.
	// This method is slower than MCG because we have to compute the preconditioning 
	// matrix P, but it can improve convergence.
	void Solve_MPCG(b3DenseVec3& x, const b3SparseMat33& A, b3DenseVec3& f, u32& iterations, const b3DenseVec3& b) const;

	b3SpringCloth * m_cloth;
	float32 m_h;
	b3Mat33* m_Jx;
	b3Mat33* m_Jv;
	u32 m_iterations;

	b3StackAllocator* m_allocator;

	b3Vec3* m_x;
	b3Vec3* m_v;
	b3Vec3* m_f;
	float32* m_m;
	float32* m_inv_m;
	b3Vec3* m_y;
	b3MassType* m_types;
	u32 m_massCount;

	b3MassContact* m_contacts;
	
	b3Spring* m_springs;
	u32 m_springCount;
};

inline u32 b3SpringSolver::GetIterations() const
{
	return m_iterations;
}

#endif