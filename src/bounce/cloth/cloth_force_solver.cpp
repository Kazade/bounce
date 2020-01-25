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

#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/forces/force.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/diag_mat33.h>
#include <bounce/sparse/sparse_mat33.h>
#include <bounce/sparse/sparse_mat33_view.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

u32 b3_clothSolverIterations = 0;

b3ClothForceSolver::b3ClothForceSolver(const b3ClothForceSolverDef& def)
{
	m_step = def.step;
	m_stack = def.stack;

	m_particleCount = def.particleCount;
	m_particles = def.particles;

	m_forceCount = def.forceCount;
	m_forces = def.forces;
}

b3ClothForceSolver::~b3ClothForceSolver()
{
}

void b3ClothForceSolver::ApplyForces()
{
	for (u32 i = 0; i < m_forceCount; ++i)
	{
		m_forces[i]->Apply(&m_solverData);
	}
}

// Solve A * x = b
static void b3SolveMPCG(b3DenseVec3& x,
	const b3SparseMat33View& A, const b3DenseVec3& b,
	const b3DenseVec3& z, const b3DiagMat33& S, u32 maxIterations = 20)
{
	B3_PROFILE("Cloth Solve MPCG");

	// Jacobi preconditioner
	// P = diag(A) 
	b3DiagMat33 P(A.rowCount);
	b3DiagMat33 invP(A.rowCount);

	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3Mat33 a = A(i, i);

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(a.x, a.y, a.z) > scalar(0));

		B3_ASSERT(a.x.x > scalar(0));
		scalar xx = scalar(1) / a.x.x;

		B3_ASSERT(a.y.y > scalar(0));
		scalar yy = scalar(1) / a.y.y;

		B3_ASSERT(a.z.z > scalar(0));
		scalar zz = scalar(1) / a.z.z;

		P[i] = b3Diagonal(a.x.x, a.y.y, a.z.z);
		invP[i] = b3Diagonal(xx, yy, zz);
	}

	x = z;

	scalar delta_0 = b3Dot(S * b, P * (S * b));

	b3DenseVec3 r = S * (b - A * x);
	b3DenseVec3 c = S * (invP * r);

	scalar delta_new = b3Dot(r, c);

	u32 iteration = 0;
	for (;;)
	{
		if (iteration == maxIterations)
		{
			break;
		}

		if (delta_new <= B3_EPSILON * B3_EPSILON * delta_0)
		{
			break;
		}

		b3DenseVec3 q = S * (A * c);

		scalar alpha = delta_new / b3Dot(c, q);

		x = x + alpha * c;
		r = r - alpha * q;

		b3DenseVec3 s = invP * r;

		scalar delta_old = delta_new;

		delta_new = b3Dot(r, s);

		scalar beta = delta_new / delta_old;

		c = S * (s + beta * c);

		++iteration;
	}

	b3_clothSolverIterations = iteration;
}

void b3ClothForceSolver::Solve(const b3Vec3& gravity)
{
	scalar h = m_step.dt;

	b3DenseVec3 sx(m_particleCount);
	b3DenseVec3 sv(m_particleCount);
	b3DenseVec3 sf(m_particleCount);
	b3DenseVec3 sy(m_particleCount);
	b3DenseVec3 sz(m_particleCount);
	b3DiagMat33 M(m_particleCount);
	b3SparseMat33 dfdx(m_particleCount);
	b3SparseMat33 dfdv(m_particleCount);
	b3DiagMat33 S(m_particleCount);

	m_solverData.x = &sx;
	m_solverData.v = &sv;
	m_solverData.f = &sf;
	m_solverData.y = &sy;
	m_solverData.z = &sz;
	m_solverData.dfdx = &dfdx;
	m_solverData.dfdv = &dfdv;
	m_solverData.S = &S;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3ClothParticle* p = m_particles[i];

		sx[i] = p->m_position;
		sv[i] = p->m_velocity;
		sf[i] = p->m_force;
		sz[i].SetZero();

		if (p->m_type == e_dynamicClothParticle)
		{
			B3_ASSERT(p->m_mass > scalar(0));
			M[i] = b3Diagonal(p->m_mass);
			S[i].SetIdentity();

			// Apply weight
			sf[i] += p->m_mass * gravity;
		}
		else
		{
			// Ensure a non-zero mass because zero masses 
			// can make the system unsolvable.
			M[i] = b3Diagonal(scalar(1));
			S[i].SetZero();
		}

		sy[i] = p->m_translation;
	}

	// Apply internal forces
	ApplyForces();

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 
	
	// A
	b3SparseMat33 A = M - h * dfdv - (h * h) * dfdx;

	// View for A
	b3SparseMat33View AV(A);

	// b
	b3DenseVec3 b = h * (sf + h * (dfdx * sv) + dfdx * sy);

	// x
	b3DenseVec3 x(m_particleCount);
	b3SolveMPCG(x, AV, b, sz, S);

	// Velocity update
	sv = sv + x;
	sx = sx + sy;

	// Copy state buffers back to the particle
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3ClothParticle* p = m_particles[i];

		p->m_x = x[i];
		p->m_position = sx[i];
		p->m_velocity = sv[i];
	}
}