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

#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/sparse_mat33.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Some improvements for the original MPCG algorithm are described in the paper:
// "On the modified conjugate gradient method in cloth simulation - Uri M. Ascher, Eddy Boxerman".

u32 b3_clothSolverIterations = 0;

b3ClothSolver::b3ClothSolver(const b3ClothSolverDef& def)
{
	m_allocator = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3Particle**)m_allocator->Allocate(m_particleCapacity * sizeof(b3Particle*));

	m_springCapacity = def.springCapacity;
	m_springCount = 0;
	m_springs = (b3Spring**)m_allocator->Allocate(m_springCapacity * sizeof(b3Spring*));;

	m_contactCapacity = def.contactCapacity;
	m_contactCount = 0;
	m_contacts = (b3ParticleContact**)m_allocator->Allocate(m_contactCapacity * sizeof(b3ParticleContact*));
	
	m_constraintCapacity = def.particleCapacity;
	m_constraintCount = 0;
	m_constraints = (b3AccelerationConstraint*)m_allocator->Allocate(m_constraintCapacity * sizeof(b3AccelerationConstraint));
}

b3ClothSolver::~b3ClothSolver()
{
	m_allocator->Free(m_constraints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_springs);
	m_allocator->Free(m_particles);
}

void b3ClothSolver::Add(b3Particle* p)
{
	p->solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3ClothSolver::Add(b3ParticleContact* c)
{
	m_contacts[m_contactCount++] = c;
}

void b3ClothSolver::Add(b3Spring* s)
{
	m_springs[m_springCount++] = s;
}

void b3ClothSolver::InitializeConstraints()
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		if (p->type == e_staticParticle)
		{
			b3AccelerationConstraint* ac = m_constraints + m_constraintCount;
			++m_constraintCount;
			ac->i1 = i;
			ac->ndof = 0;
			ac->z.SetZero();
		}
	}

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ParticleContact* pc = m_contacts[i];
		b3Particle* p = pc->p1;

		B3_ASSERT(p->type != e_staticParticle);

		b3AccelerationConstraint* ac = m_constraints + m_constraintCount;
		++m_constraintCount;
		ac->i1 = p->solverId;
		ac->ndof = 2;
		ac->p = pc->n;
		ac->z.SetZero();

		if (pc->t1_active && pc->t2_active)
		{
			ac->ndof = 0;
		}
		else
		{
			if (pc->t1_active)
			{
				ac->ndof = 1;
				ac->q = pc->t1;
			}
			
			if (pc->t2_active)
			{
				ac->ndof = 1;
				ac->q = pc->t2;
			}
		}
	}
}

static B3_FORCE_INLINE b3SparseMat33 b3AllocSparse(b3StackAllocator* allocator, u32 M, u32 N)
{
	u32 size = M * N;
	b3Mat33* elements = (b3Mat33*)allocator->Allocate(size * sizeof(b3Mat33));
	u32* cols = (u32*)allocator->Allocate(size * sizeof(u32));
	u32* row_ptrs = (u32*)allocator->Allocate((M + 1) * sizeof(u32));

	return b3SparseMat33(M, N, size, elements, row_ptrs, cols);
}

static B3_FORCE_INLINE void b3FreeSparse(b3SparseMat33& matrix, b3StackAllocator* allocator)
{
	allocator->Free(matrix.row_ptrs);
	allocator->Free(matrix.cols);
	allocator->Free(matrix.values);
}

void b3ClothSolver::Solve(float32 dt, const b3Vec3& gravity)
{
	B3_PROFILE("Integrate");

	b3DenseVec3 sx(m_particleCount);
	b3DenseVec3 sv(m_particleCount);
	b3DenseVec3 sf(m_particleCount);
	b3DenseVec3 sy(m_particleCount);
	b3DenseVec3 sx0(m_particleCount);

	m_solverData.x = sx.v;
	m_solverData.v = sv.v;
	m_solverData.f = sf.v;
	m_solverData.dt = dt;
	m_solverData.invdt = 1.0f / dt;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		sx[i] = p->position;
		sv[i] = p->velocity;
		sf[i] = p->force;

		// Apply weight
		if (p->type == e_dynamicParticle)
		{
			sf[i] += p->mass * gravity;
		}

		sy[i] = p->translation;
		sx0[i] = p->x;
	}

	// Apply contact position corrections
	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ParticleContact* c = m_contacts[i];
		b3Particle* p = c->p1;	
		sy[p->solverId] -= c->s * c->n;
	}

	// Apply spring forces and derivatives
	for (u32 i = 0; i < m_springCount; ++i)
	{
		m_springs[i]->ApplyForces(&m_solverData);
	}

	// Initialize constraints
	InitializeConstraints();

	b3DiagMat33 S(m_particleCount);
	b3DenseVec3 z(m_particleCount);
	Compute_S_z(S, z);

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	// A
	b3SparseMat33 A = b3AllocSparse(m_allocator, m_particleCount, m_particleCount);

	// b
	b3DenseVec3 b(m_particleCount);

	Compute_A_b(A, b, sf, sx, sv, sy);

	// x
	b3DenseVec3 x(m_particleCount);

	// Solve Ax = b
	u32 iterations = 0;
	Solve(x, iterations, A, b, S, z, sx0);
	b3_clothSolverIterations = iterations;

	// f = A * x - b
	b3DenseVec3 f = A * x - b;

	// Update state
	float32 h = dt;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		b3ParticleType type = p->type;

		b3Vec3 ix0 = sx[i];
		b3Vec3 iv0 = sv[i];
		b3Vec3 iy = sy[i];

		b3Vec3 dv = x[i];

		// v1 = v0 + dv
		b3Vec3 v1 = iv0;
		if (type == e_dynamicParticle)
		{
			v1 += dv;
		}

		// dx = h * (v0 + dv) + y = h * v1 + y
		b3Vec3 dx = h * v1 + iy;

		// x1 = x0 + dx
		b3Vec3 x1 = ix0 + dx;

		sv[i] = v1;
		sx[i] = x1;
	}

	// Cache x to improve convergence
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->x = x[i];
	}

	// Store the extra contact constraint forces that should have been 
	// supplied to enforce the contact constraints exactly.
	// These forces can be used in contact constraint logic.
	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ParticleContact* c = m_contacts[i];
		b3Particle* p = c->p1;

		b3Vec3 force = f[p->solverId];

		// Signed normal force magnitude
		c->Fn = b3Dot(force, c->n);

		// Signed tangent force magnitude
		c->Ft1 = b3Dot(force, c->t1);
		c->Ft2 = b3Dot(force, c->t2);
	}

	// Copy state buffers back to the particles
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->position = sx[i];
		m_particles[i]->velocity = sv[i];
	}

	b3FreeSparse(A, m_allocator);
}

#define B3_INDEX(i, j, size) (i + j * size)

//
static void b3SetZero(b3Mat33* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

// dfdx * v
static void b3Mul_Jx(b3DenseVec3& out, b3Spring** springs, u32 springCount, const b3DenseVec3& v)
{
	out.SetZero();

	for (u32 i = 0; i < springCount; ++i)
	{
		b3Spring* s = springs[i];
		u32 i1 = s->p1->solverId;
		u32 i2 = s->p2->solverId;

		b3Mat33 J_11 = s->Jx;
		b3Mat33 J_12 = -J_11;
		b3Mat33 J_21 = J_12;
		b3Mat33 J_22 = J_11;

		out[i1] += J_11 * v[i1] + J_12 * v[i2];
		out[i2] += J_21 * v[i1] + J_22 * v[i2];
	}
}

static B3_FORCE_INLINE bool b3IsZero(const b3Mat33& A)
{
	bool isZeroX = b3Dot(A.x, A.x) == 0.0f;
	bool isZeroY = b3Dot(A.y, A.y) == 0.0f;
	bool isZeroZ = b3Dot(A.z, A.z) == 0.0f;

	return isZeroX * isZeroY * isZeroZ;
}

void b3ClothSolver::Compute_A_b(b3SparseMat33& SA, b3DenseVec3& b, const b3DenseVec3& f, const b3DenseVec3& x, const b3DenseVec3& v, const b3DenseVec3& y) const
{
	float32 h = m_solverData.dt;

	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)m_allocator->Allocate(m_particleCount * m_particleCount * sizeof(b3Mat33));
	b3SetZero(dfdx, m_particleCount * m_particleCount);

	b3Mat33* dfdv = (b3Mat33*)m_allocator->Allocate(m_particleCount * m_particleCount * sizeof(b3Mat33));
	b3SetZero(dfdv, m_particleCount * m_particleCount);

	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* s = m_springs[i];
		u32 i1 = s->p1->solverId;
		u32 i2 = s->p2->solverId;

		b3Mat33 Jx11 = s->Jx;
		b3Mat33 Jx12 = -Jx11;
		b3Mat33 Jx21 = Jx12;
		b3Mat33 Jx22 = Jx11;

		dfdx[B3_INDEX(i1, i1, m_particleCount)] += Jx11;
		dfdx[B3_INDEX(i1, i2, m_particleCount)] += Jx12;
		dfdx[B3_INDEX(i2, i1, m_particleCount)] += Jx21;
		dfdx[B3_INDEX(i2, i2, m_particleCount)] += Jx22;

		b3Mat33 Jv11 = s->Jv;
		b3Mat33 Jv12 = -Jv11;
		b3Mat33 Jv21 = Jv12;
		b3Mat33 Jv22 = Jv11;

		dfdv[B3_INDEX(i1, i1, m_particleCount)] += Jv11;
		dfdv[B3_INDEX(i1, i2, m_particleCount)] += Jv12;
		dfdv[B3_INDEX(i2, i1, m_particleCount)] += Jv21;
		dfdv[B3_INDEX(i2, i2, m_particleCount)] += Jv22;
	}

	// Compute A
	
	// A = M - h * dfdv - h * h * dfdx

	// A = 0
	b3Mat33* A = (b3Mat33*)m_allocator->Allocate(m_particleCount * m_particleCount * sizeof(b3Mat33));
	b3SetZero(A, m_particleCount * m_particleCount);

	// A += M
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		A[B3_INDEX(i, i, m_particleCount)] += b3Diagonal(m_particles[i]->mass);
	}

	// A += - h * dfdv - h * h * dfdx
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		for (u32 j = 0; j < m_particleCount; ++j)
		{
			A[B3_INDEX(i, j, m_particleCount)] += (-h * dfdv[B3_INDEX(i, j, m_particleCount)]) + (-h * h * dfdx[B3_INDEX(i, j, m_particleCount)]);
		}
	}

	// Assembly sparsity
	u32 nzCount = 0;

	SA.row_ptrs[0] = 0;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		u32 rowNzCount = 0;

		for (u32 j = 0; j < m_particleCount; ++j)
		{
			b3Mat33 a = A[B3_INDEX(i, j, m_particleCount)];

			if (b3IsZero(a) == false)
			{
				B3_ASSERT(nzCount <= SA.valueCount);

				SA.values[nzCount] = a;
				SA.cols[nzCount] = j;

				++nzCount;
				++rowNzCount;
			}
		}

		SA.row_ptrs[i + 1] = SA.row_ptrs[(i + 1) - 1] + rowNzCount;
	}

	B3_ASSERT(nzCount <= SA.valueCount);
	SA.valueCount = nzCount;

	m_allocator->Free(A);

	m_allocator->Free(dfdv);
	m_allocator->Free(dfdx);

	// Compute b

	// Jx_v = dfdx * v
	b3DenseVec3 Jx_v(m_particleCount);
	b3Mul_Jx(Jx_v, m_springs, m_springCount, v);

	// Jx_y = dfdx * y
	b3DenseVec3 Jx_y(m_particleCount);
	b3Mul_Jx(Jx_y, m_springs, m_springCount, y);

	// b = h * (f0 + h * Jx_v + Jx_y)
	b = h * (f + h * Jx_v + Jx_y);
}

void b3ClothSolver::Compute_S_z(b3DiagMat33& S, b3DenseVec3& z)
{
	S.SetIdentity();
	z.SetZero();

	for (u32 i = 0; i < m_constraintCount; ++i)
	{
		b3AccelerationConstraint* ac = m_constraints + i;
		u32 ip = ac->i1;
		u32 ndof = ac->ndof;
		b3Vec3 p = ac->p;
		b3Vec3 q = ac->q;
		b3Vec3 cz = ac->z;

		z[ip] = cz;

		if (ndof == 2)
		{
			b3Mat33 I; I.SetIdentity();

			S[ip] = I - b3Outer(p, p);
		}

		if (ndof == 1)
		{
			b3Mat33 I; I.SetIdentity();

			S[ip] = I - b3Outer(p, p) - b3Outer(q, q);
		}

		if (ndof == 0)
		{
			S[ip].SetZero();
		}
	}
}

void b3ClothSolver::Solve(b3DenseVec3& x, u32& iterations,
	const b3SparseMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const
{
	B3_PROFILE("Solve Ax = b");

	// P = diag(A)
	b3DiagMat33 inv_P(m_particleCount);
	A.AssembleDiagonal(inv_P);

	// Invert
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Mat33& D = inv_P[i];

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(D.x, D.y, D.z) > 0.0f);

		B3_ASSERT(D.x.x != 0.0f);
		float32 xx = 1.0f / D.x.x;

		B3_ASSERT(D.y.y != 0.0f);
		float32 yy = 1.0f / D.y.y;

		B3_ASSERT(D.z.z != 0.0f);
		float32 zz = 1.0f / D.z.z;

		D = b3Diagonal(xx, yy, zz);
	}

	// I
	b3DiagMat33 I(m_particleCount);
	I.SetIdentity();

	// x = S * y + (I - S) * z 
	x = (S * y) + (I - S) * z;

	// b^ = S * (b - A * (I - S) * z)
	b3DenseVec3 b_hat = S * (b - A * ((I - S) * z));

	// b_delta = dot(b^, P^-1 * b_^)
	float32 b_delta = b3Dot(b_hat, inv_P * b_hat);

	// r = S * (b - A * x)
	b3DenseVec3 r = S * (b - A * x);

	// p = S * (P^-1 * r)
	b3DenseVec3 p = S * (inv_P * r);

	// deltaNew = dot(r, p)
	float32 deltaNew = b3Dot(r, p);

	// Tolerance.
	// This is the main stopping criteria.
	// [0, 1]
	const float32 tolerance = 10.0f * B3_EPSILON;

	// Maximum number of iterations.
	const u32 maxIters = 1000;

	// Main iteration loop.
	u32 iter = 0;
	while (deltaNew > tolerance * tolerance * b_delta && iter < maxIters)
	{
		// s = S * (A * p)
		b3DenseVec3 s = S * (A * p);

		// alpha = deltaNew / dot(c, q)
		float32 alpha = deltaNew / b3Dot(p, s);

		// x = x + alpha * p
		x = x + alpha * p;

		// r = r - alpha * s
		r = r - alpha * s;

		// h = inv_P * r
		b3DenseVec3 h = inv_P * r;

		// deltaOld = deltaNew
		float32 deltaOld = deltaNew;

		// deltaNew = dot(r, h)
		deltaNew = b3Dot(r, h);
		//B3_ASSERT(b3IsValid(deltaNew));

		// beta = deltaNew / deltaOld
		float32 beta = deltaNew / deltaOld;

		// p = S * (h + beta * p)
		p = S * (h + beta * p);

		++iter;
	}

	iterations = iter;
}