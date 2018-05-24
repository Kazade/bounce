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
	m_Jx = (b3Mat33*)m_allocator->Allocate(m_springCapacity * sizeof(b3Mat33));
	m_Jv = (b3Mat33*)m_allocator->Allocate(m_springCapacity * sizeof(b3Mat33));

	m_contactCapacity = def.contactCapacity;
	m_contactCount = 0;
	m_contacts = (b3ParticleContact**)m_allocator->Allocate(m_contactCapacity * sizeof(b3ParticleContact*));;
}

b3ClothSolver::~b3ClothSolver()
{
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_Jv);
	m_allocator->Free(m_Jx);
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

static B3_FORCE_INLINE void b3CopyPosition(b3DenseVec3& v, b3Particle** const particles, u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		v[i] = particles[i]->position;
	}
}

static B3_FORCE_INLINE void b3CopyVelocity(b3DenseVec3& v, b3Particle** const particles, u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		v[i] = particles[i]->velocity;
	}
}

static B3_FORCE_INLINE void b3CopyForce(b3DenseVec3& v, b3Particle** const particles, u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		v[i] = particles[i]->force;
	}
}

static B3_FORCE_INLINE void b3CopyTranslation(b3DenseVec3& v, b3Particle** const particles, u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		v[i] = particles[i]->translation;
	}
}

static B3_FORCE_INLINE void b3CopyGuess(b3DenseVec3& v, b3Particle** const particles, u32 count)
{
	for (u32 i = 0; i < count; ++i)
	{
		v[i] = particles[i]->x;
	}
}

void b3ClothSolver::Solve(float32 dt, const b3Vec3& gravity)
{
	B3_PROFILE("Integrate");

	m_h = dt;

	b3DenseVec3 sx(m_particleCount);
	b3CopyPosition(sx, m_particles, m_particleCount);

	b3DenseVec3 sv(m_particleCount);
	b3CopyVelocity(sv, m_particles, m_particleCount);

	b3DenseVec3 sf(m_particleCount);
	b3CopyForce(sf, m_particles, m_particleCount);

	b3DenseVec3 sy(m_particleCount);
	b3CopyTranslation(sy, m_particles, m_particleCount);

	Compute_f(sf, sx, sv, gravity);

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	// A
	b3SparseMat33 A = b3AllocSparse(m_allocator, m_particleCount, m_particleCount);

	// b
	b3DenseVec3 b(m_particleCount);

	Compute_A_b(A, b, sf, sx, sv, sy);

	// S
	b3DiagMat33 S(m_particleCount);
	Compute_S(S);

	// z
	b3DenseVec3 z(m_particleCount);
	Compute_z(z);

	// x0
	b3DenseVec3 x0(m_particleCount);
	b3CopyGuess(x0, m_particles, m_particleCount);

	// x
	b3DenseVec3 x(m_particleCount);

	// Solve Ax = b
	u32 iterations = 0;
	Solve(x, iterations, A, b, S, z, x0);

	b3_clothSolverIterations = iterations;

	// f = A * x - b
	b3DenseVec3 f = A * x - b;

	// Update state
	float32 h = m_h;

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

	// Write x to the solution cache for improving convergence
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->x = x[i];
	}

	// Store the extra contact constraint forces that should have been 
	// supplied in order to enforce the contact constraints exactly
	// These forces can be used in contact constraint logic
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
static void b3SetZero(b3Vec3* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

//
static void b3SetZero(b3Mat33* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

// 
static void b3Mul_Jacobian(b3Vec3* out, const b3Vec3* v, u32 massCount,
	const b3Mat33* J_ii, b3Spring** const springs, u32 springCount)
{
	b3SetZero(out, massCount);

	for (u32 i = 0; i < springCount; ++i)
	{
		const b3Spring* S = springs[i];
		u32 i1 = S->p1->solverId;
		u32 i2 = S->p2->solverId;

		b3Mat33 J_11 = J_ii[i];
		b3Mat33 J_12 = -J_11;
		b3Mat33 J_21 = J_12;
		b3Mat33 J_22 = J_11;

		out[i1] += J_11 * v[i1] + J_12 * v[i2];
		out[i2] += J_21 * v[i1] + J_22 * v[i2];
	}
}

void b3ClothSolver::Compute_f(b3DenseVec3& f, const b3DenseVec3& x, const b3DenseVec3& v, const b3Vec3& gravity)
{
	// Zero force
	f.SetZero();

	// Apply weight
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		if (p->type == e_dynamicParticle)
		{
			f[i] += p->mass * gravity;
		}
	}

	// Apply tension, damping
	// Store the Jacobians

	// Zero Jacobians
	b3SetZero(m_Jx, m_springCount);
	b3SetZero(m_Jv, m_springCount);

	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* S = m_springs[i];
		b3SpringType type = S->type;
		b3Particle* p1 = S->p1;
		b3Particle* p2 = S->p2;
		float32 L0 = S->L0;
		float32 ks = S->ks;
		float32 kd = S->kd;

		u32 i1 = p1->solverId;
		u32 i2 = p2->solverId;

		b3Vec3 x1 = x[i1];
		b3Vec3 v1 = v[i1];

		b3Vec3 x2 = x[i2];
		b3Vec3 v2 = v[i2];

		const b3Mat33 I = b3Mat33_identity;

		b3Vec3 dx = x1 - x2;

		if (b3Dot(dx, dx) >= L0 * L0)
		{
			// Tension
			float32 L = b3Length(dx);
			b3Vec3 n = dx / L;

			b3Vec3 sf1 = -ks * (L - L0) * n;
			b3Vec3 sf2 = -sf1;

			f[i1] += sf1;
			f[i2] += sf2;

			p1->tension += sf1;
			p2->tension += sf2;

			b3Mat33 Jx11 = -ks * (b3Outer(dx, dx) + (1.0f - L0 / L) * (I - b3Outer(dx, dx)));

			m_Jx[i] = Jx11;
		}

		// Damping
		b3Vec3 dv = v1 - v2;

		b3Vec3 df1 = -kd * dv;
		b3Vec3 df2 = -df1;

		f[i1] += df1;
		f[i2] += df2;

		b3Mat33 Jv11 = -kd * I;

		m_Jv[i] = Jv11;
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
	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)m_allocator->Allocate(m_particleCount * m_particleCount * sizeof(b3Mat33));
	b3SetZero(dfdx, m_particleCount * m_particleCount);

	b3Mat33* dfdv = (b3Mat33*)m_allocator->Allocate(m_particleCount * m_particleCount * sizeof(b3Mat33));
	b3SetZero(dfdv, m_particleCount * m_particleCount);

	for (u32 i = 0; i < m_springCount; ++i)
	{
		const b3Spring* S = m_springs[i];
		u32 i1 = S->p1->solverId;
		u32 i2 = S->p2->solverId;

		b3Mat33 Jx11 = m_Jx[i];
		b3Mat33 Jx12 = -Jx11;
		b3Mat33 Jx21 = Jx12;
		b3Mat33 Jx22 = Jx11;

		dfdx[B3_INDEX(i1, i1, m_particleCount)] += Jx11;
		dfdx[B3_INDEX(i1, i2, m_particleCount)] += Jx12;
		dfdx[B3_INDEX(i2, i1, m_particleCount)] += Jx21;
		dfdx[B3_INDEX(i2, i2, m_particleCount)] += Jx22;

		b3Mat33 Jv11 = m_Jv[i];
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
			A[B3_INDEX(i, j, m_particleCount)] += (-m_h * dfdv[B3_INDEX(i, j, m_particleCount)]) + (-m_h * m_h * dfdx[B3_INDEX(i, j, m_particleCount)]);
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
	// b = h * (f0 + h * Jx_v + Jx_y)

	// Jx_v = dfdx * v
	b3Vec3* Jx_v = (b3Vec3*)m_allocator->Allocate(m_particleCount * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_v, v.v, m_particleCount, m_Jx, m_springs, m_springCount);

	// Jx_y = dfdx * y
	b3Vec3* Jx_y = (b3Vec3*)m_allocator->Allocate(m_particleCount * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_y, y.v, m_particleCount, m_Jx, m_springs, m_springCount);

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b[i] = m_h * (f[i] + m_h * Jx_v[i] + Jx_y[i]);
	}

	m_allocator->Free(Jx_y);
	m_allocator->Free(Jx_v);
}

void b3ClothSolver::Compute_z(b3DenseVec3& z)
{
	// TODO
	z.SetZero();
}

void b3ClothSolver::Compute_S(b3DiagMat33& out)
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		if (p->type == e_staticParticle)
		{
			out[i].SetZero();
			continue;
		}

		out[i].SetIdentity();
	}

	for (u32 i = 0; i < m_contactCount; ++i)
	{
		b3ParticleContact* c = m_contacts[i];
		
		B3_ASSERT(c->n_active);

		b3Vec3 n = c->n;

		b3Mat33 S = b3Mat33_identity - b3Outer(n, n);

		if (c->t1_active == true && c->t2_active == true)
		{
			S.SetZero();
		}
		else
		{
			if (c->t1_active == true)
			{
				b3Vec3 t1 = c->t1;

				S -= b3Outer(t1, t1);
			}

			if (c->t2_active == true)
			{
				b3Vec3 t2 = c->t2;

				S -= b3Outer(t2, t2);
			}
		}

		b3Particle* p = c->p1;
		out[p->solverId] = S;
	}
}

void b3ClothSolver::Solve(b3DenseVec3& x, u32& iterations,
	const b3SparseMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const
{
	B3_PROFILE("Solve A * x = b");

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