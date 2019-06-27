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
#include <bounce/cloth/particle.h>
#include <bounce/cloth/force.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/diag_mat33.h>
#include <bounce/sparse/sparse_mat33.h>
#include <bounce/sparse/sparse_mat33_view.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Some improvements for the original MPCG algorithm are described in the paper:
// "On the modified conjugate gradient method in cloth simulation - Uri M. Ascher, Eddy Boxerman".

u32 b3_clothSolverIterations = 0;

b3ClothForceSolver::b3ClothForceSolver(const b3ClothForceSolverDef& def)
{
	m_allocator = def.stack;

	m_particleCount = def.particleCount;
	m_particles = def.particles;

	m_forceCount = def.forceCount;
	m_forces = def.forces;

	m_constraints = (b3AccelerationConstraint*)m_allocator->Allocate(m_particleCount * sizeof(b3AccelerationConstraint));
}

b3ClothForceSolver::~b3ClothForceSolver()
{
	m_allocator->Free(m_constraints);
}

void b3ClothForceSolver::ApplyForces()
{
	for (u32 i = 0; i < m_forceCount; ++i)
	{
		m_forces[i]->Apply(&m_solverData);
	}
}

void b3AccelerationConstraint::Apply(const b3ClothForceSolverData* data)
{
	b3DiagMat33& sS = *data->S;
	b3DenseVec3& sz = *data->z;

	sz[i1] = z;

	b3Mat33 I; I.SetIdentity();

	switch (ndof)
	{
	case 3:
	{
		sS[i1] = I;
		break;
	}
	case 2:
	{
		sS[i1] = I - b3Outer(p, p);
		break;
	}
	case 1:
	{
		sS[i1] = I - b3Outer(p, p) - b3Outer(q, q);
		break;
	}
	case 0:
	{
		sS[i1].SetZero();
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

void b3ClothForceSolver::ApplyConstraints()
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		b3AccelerationConstraint* c = m_constraints + i;

		c->i1 = i;

		if (p->m_type != e_dynamicParticle)
		{
			c->ndof = 0;
			c->z.SetZero();
		}
		else
		{
			c->ndof = 3;
			c->z.SetZero();
		}
	}

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_constraints[i].Apply(&m_solverData);
	}
}

// Solve Ax = b
static void b3SolveMPCG(b3DenseVec3& x,
	const b3SparseMat33View& A, const b3DenseVec3& b,
	const b3DiagMat33& S, const b3DenseVec3& z,
	const b3DenseVec3& y, const b3DiagMat33& I, u32 maxIterations = 20)
{
	B3_PROFILE("Cloth Solve MPCG");

	// Jacobi preconditioner 
	// P = diag(A)
	b3DiagMat33 inv_P(A.rowCount);
	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3Mat33 a = A(i, i);

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(a.x, a.y, a.z) > 0.0f);

		B3_ASSERT(a.x.x > 0.0f);
		float32 xx = 1.0f / a.x.x;

		B3_ASSERT(a.y.y > 0.0f);
		float32 yy = 1.0f / a.y.y;

		B3_ASSERT(a.z.z > 0.0f);
		float32 zz = 1.0f / a.z.z;

		inv_P[i] = b3Diagonal(xx, yy, zz);
	}

	x = (S * y) + (I - S) * z;

	b3DenseVec3 b_hat = S * (b - A * ((I - S) * z));

	float32 b_delta = b3Dot(b_hat, inv_P * b_hat);

	b3DenseVec3 r = S * (b - A * x);

	b3DenseVec3 p = S * (inv_P * r);

	float32 delta_new = b3Dot(r, p);

	u32 iteration = 0;
	for (;;)
	{
		if (iteration == maxIterations)
		{
			break;
		}

		if (delta_new <= B3_EPSILON * B3_EPSILON * b_delta)
		{
			break;
		}

		b3DenseVec3 s = S * (A * p);

		float32 alpha = delta_new / b3Dot(p, s);

		x = x + alpha * p;
		r = r - alpha * s;

		b3DenseVec3 h = inv_P * r;

		float32 delta_old = delta_new;

		delta_new = b3Dot(r, h);

		float32 beta = delta_new / delta_old;

		p = S * (h + beta * p);

		++iteration;
	}

	b3_clothSolverIterations = iteration;
}

void b3ClothForceSolver::Solve(float32 dt, const b3Vec3& gravity)
{
	float32 h = dt;

	b3DenseVec3 sx(m_particleCount);
	b3DenseVec3 sv(m_particleCount);
	b3DenseVec3 sf(m_particleCount);
	b3DenseVec3 sy(m_particleCount);
	b3DenseVec3 sz(m_particleCount);
	b3DenseVec3 sx0(m_particleCount);
	b3SparseMat33 M(m_particleCount);
	b3SparseMat33 dfdx(m_particleCount);
	b3SparseMat33 dfdv(m_particleCount);
	b3DiagMat33 S(m_particleCount);
	b3DiagMat33 I(m_particleCount);
	I.SetIdentity();

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
		b3Particle* p = m_particles[i];

		M(i, i) = b3Diagonal(p->m_mass);
		
		sx[i] = p->m_position;
		sv[i] = p->m_velocity;
		sf[i] = p->m_force;

		if (p->m_type == e_dynamicParticle)
		{
			// Apply weight
			sf[i] += p->m_mass * gravity;
		}

		sy[i] = p->m_translation;
		sx0[i] = p->m_x;
	}

	// Apply internal forces
	ApplyForces();

	// Apply constraints
	ApplyConstraints();

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 
	
	// A
	b3SparseMat33 A = M - h * dfdv - (h * h) * dfdx;

	// View for A
	b3SparseMat33View viewA(A);

	// b
	b3DenseVec3 b = h * (sf + h * (dfdx * sv) + dfdx * sy);

	// x
	b3DenseVec3 x(m_particleCount);
	b3SolveMPCG(x, viewA, b, S, sz, sx0, I);

	// Velocity update
	sv = sv + x;
	sx = sx + sy;

	// Copy state buffers back to the particle
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		p->m_x = x[i];
		p->m_position = sx[i];
		p->m_velocity = sv[i];
	}
}