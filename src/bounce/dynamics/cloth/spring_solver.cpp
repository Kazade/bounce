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

#include <bounce/dynamics/cloth/spring_solver.h>
#include <bounce/dynamics/cloth/spring_cloth.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/sparse_mat33.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Some improvements for the original MPCG algorithm are described in the paper:
// "On the modified conjugate gradient method in cloth simulation - Uri M. Ascher, Eddy Boxerman".

b3SpringSolver::b3SpringSolver(const b3SpringSolverDef& def)
{
	m_cloth = def.cloth;
	m_h = def.dt;
	m_iterations = 0;
	m_Jx = nullptr;
	m_Jv = nullptr;

	m_allocator = m_cloth->m_allocator;

	m_x = m_cloth->m_x;
	m_v = m_cloth->m_v;
	m_f = m_cloth->m_f;
	m_m = m_cloth->m_m;
	m_inv_m = m_cloth->m_inv_m;
	m_y = m_cloth->m_y;
	m_z = m_cloth->m_y;
	m_x0 = m_cloth->m_x0;
	m_types = m_cloth->m_types;
	m_massCount = m_cloth->m_massCount;

	m_contacts = m_cloth->m_contacts;

	m_springs = m_cloth->m_springs;
	m_springCount = m_cloth->m_springCount;

	m_shapes = m_cloth->m_shapes;
}

b3SpringSolver::~b3SpringSolver()
{

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

void b3SpringSolver::Solve(b3DenseVec3& f)
{
	B3_PROFILE("Solve");

	//
	m_Jx = (b3Mat33*)m_allocator->Allocate(m_springCount * sizeof(b3Mat33));
	m_Jv = (b3Mat33*)m_allocator->Allocate(m_springCount * sizeof(b3Mat33));

	// Apply internal forces. Also, store their unique derivatives.
	ApplySpringForces();

	// Integrate

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	{
		// A
		b3SparseMat33 A = b3AllocSparse(m_allocator, m_massCount, m_massCount);

		// b
		b3DenseVec3 b(m_massCount);

		Compute_A_b(A, b);

		// S
		b3DiagMat33 S(m_massCount);
		Compute_S(S);

		// z
		b3DenseVec3 z(m_massCount);
		Compute_z(z);

		// x0
		b3DenseVec3 x0(m_massCount);
		for (u32 i = 0; i < m_massCount; ++i)
		{
			x0[i] = m_x0[i];
		}

		// x
		b3DenseVec3 x(m_massCount);

		// Solve Ax = b
		Solve(x, f, m_iterations, A, b, S, z, x0);

		// Store x0
		for (u32 i = 0; i < m_massCount; ++i)
		{
			m_x0[i] = x[i];
		}
		
		// Update state
		for (u32 i = 0; i < m_massCount; ++i)
		{
			if (m_types[i] == b3MassType::e_dynamicMass)
			{
				m_v[i] += x[i];
			}

			// dx = h * (v0 + dv) + y = h * v1 + y
			m_x[i] += m_h * m_v[i] + m_y[i];
		}

		b3FreeSparse(A, m_allocator);
	}

	m_allocator->Free(m_Jv);
	m_allocator->Free(m_Jx);
	m_Jv = nullptr;
	m_Jx = nullptr;
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
	const b3Mat33* J_ii, const b3Spring* springs, u32 springCount)
{
	b3SetZero(out, massCount);

	for (u32 i = 0; i < springCount; ++i)
	{
		const b3Spring* S = springs + i;
		u32 i1 = S->i1;
		u32 i2 = S->i2;

		b3Mat33 J_11 = J_ii[i];
		b3Mat33 J_12 = -J_11;
		b3Mat33 J_21 = J_12;
		b3Mat33 J_22 = J_11;

		out[i1] += J_11 * v[i1] + J_12 * v[i2];
		out[i2] += J_21 * v[i1] + J_22 * v[i2];
	}
}

void b3SpringSolver::ApplySpringForces()
{
	// Zero Jacobians
	b3SetZero(m_Jx, m_springCount);
	b3SetZero(m_Jv, m_springCount);

	// Compute spring forces and Jacobians
	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* S = m_springs + i;

		b3SpringType type = S->type;
		u32 i1 = S->i1;
		u32 i2 = S->i2;
		float32 L0 = S->L0;
		B3_ASSERT(L0 > 0.0f);
		float32 ks = S->ks;
		float32 kd = S->kd;

		b3Vec3 x1 = m_x[i1];
		b3Vec3 v1 = m_v[i1];

		b3Vec3 x2 = m_x[i2];
		b3Vec3 v2 = m_v[i2];

		const b3Mat33 I = b3Mat33_identity;

		// Strech
		b3Vec3 dx = x1 - x2;
		float32 L = b3Length(dx);

		if (L >= L0)
		{
			// Force is tension.
			b3Vec3 n = dx / L;

			b3Vec3 sf1 = -ks * (L - L0) * n;
			b3Vec3 sf2 = -sf1;

			m_f[i1] += sf1;
			m_f[i2] += sf2;

			b3Mat33 Jx11 = -ks * (b3Outer(dx, dx) + (1.0f - L0 / L) * (I - b3Outer(dx, dx)));

			m_Jx[i] = Jx11;
		}

		// Damping
		b3Vec3 dv = v1 - v2;

		b3Vec3 df1 = -kd * dv;
		b3Vec3 df2 = -df1;

		m_f[i1] += df1;
		m_f[i2] += df2;

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

void b3SpringSolver::Compute_A_b(b3SparseMat33& SA, b3DenseVec3& b) const
{
	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)m_allocator->Allocate(m_massCount * m_massCount * sizeof(b3Mat33));
	b3SetZero(dfdx, m_massCount * m_massCount);
	
	b3Mat33* dfdv = (b3Mat33*)m_allocator->Allocate(m_massCount * m_massCount * sizeof(b3Mat33));
	b3SetZero(dfdv, m_massCount * m_massCount);

	for (u32 i = 0; i < m_springCount; ++i)
	{
		const b3Spring* S = m_springs + i;
		u32 i1 = S->i1;
		u32 i2 = S->i2;

		b3Mat33 Jx11 = m_Jx[i];
		b3Mat33 Jx12 = -Jx11;
		b3Mat33 Jx21 = Jx12;
		b3Mat33 Jx22 = Jx11;

		dfdx[B3_INDEX(i1, i1, m_massCount)] += Jx11;
		dfdx[B3_INDEX(i1, i2, m_massCount)] += Jx12;
		dfdx[B3_INDEX(i2, i1, m_massCount)] += Jx21;
		dfdx[B3_INDEX(i2, i2, m_massCount)] += Jx22;

		b3Mat33 Jv11 = m_Jv[i];
		b3Mat33 Jv12 = -Jv11;
		b3Mat33 Jv21 = Jv12;
		b3Mat33 Jv22 = Jv11;

		dfdv[B3_INDEX(i1, i1, m_massCount)] += Jv11;
		dfdv[B3_INDEX(i1, i2, m_massCount)] += Jv12;
		dfdv[B3_INDEX(i2, i1, m_massCount)] += Jv21;
		dfdv[B3_INDEX(i2, i2, m_massCount)] += Jv22;
	}

	// Compute A
	// A = M - h * dfdv - h * h * dfdx

	// A = 0
	b3Mat33* A = (b3Mat33*)m_allocator->Allocate(m_massCount * m_massCount * sizeof(b3Mat33));
	b3SetZero(A, m_massCount * m_massCount);

	// A += M
	for (u32 i = 0; i < m_massCount; ++i)
	{
		A[B3_INDEX(i, i, m_massCount)] += b3Diagonal(m_m[i]);
	}
	
	// A += - h * dfdv - h * h * dfdx
	for (u32 i = 0; i < m_massCount; ++i)
	{
		for (u32 j = 0; j < m_massCount; ++j)
		{
			A[B3_INDEX(i, j, m_massCount)] += (-m_h * dfdv[B3_INDEX(i, j, m_massCount)]) + (-m_h * m_h * dfdx[B3_INDEX(i, j, m_massCount)]);
		}
	}

	// Assembly sparsity
	u32 nzCount = 0;

	SA.row_ptrs[0] = 0;

	for (u32 i = 0; i < m_massCount; ++i)
	{
		u32 rowNzCount = 0;

		for (u32 j = 0; j < m_massCount; ++j)
		{
			b3Mat33 a = A[B3_INDEX(i, j, m_massCount)];

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

	// Compute b
	// b = h * (f0 + h * Jx_v + Jx_y)

	// Jx_v = dfdx * v
	b3Vec3* Jx_v = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_v, m_v, m_massCount, m_Jx, m_springs, m_springCount);

	// Jx_y = dfdx * y
	b3Vec3* Jx_y = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_y, m_y, m_massCount, m_Jx, m_springs, m_springCount);

	for (u32 i = 0; i < m_massCount; ++i)
	{
		b[i] = m_h * (m_f[i] + m_h * Jx_v[i] + Jx_y[i]);
	}

	m_allocator->Free(Jx_y);
	m_allocator->Free(Jx_v);

	m_allocator->Free(dfdv);
	m_allocator->Free(dfdx);
}

void b3SpringSolver::Compute_z(b3DenseVec3& z)
{
	for (u32 i = 0; i < m_massCount; ++i)
	{
		z[i] = m_z[i];
	}
}

void b3SpringSolver::Compute_S(b3DiagMat33& out)
{
	for (u32 i = 0; i < m_massCount; ++i)
	{
		out[i].SetIdentity();

		if (m_types[i] == b3MassType::e_staticMass)
		{
			out[i].SetZero();
			continue;
		}

		b3MassContact* c = m_contacts + i;

		if (c->lockN == true)
		{
			b3Vec3 n = c->n;

			b3Mat33 S = b3Mat33_identity - b3Outer(n, n);

			if (c->lockT1 == true && c->lockT2 == true)
			{
				S.SetZero();
			}
			else
			{
				if (c->lockT1 == true)
				{
					b3Vec3 t1 = c->t1;

					S -= b3Outer(t1, t1);
				}

				if (c->lockT2 == true)
				{
					b3Vec3 t2 = c->t2;

					S -= b3Outer(t2, t2);
				}
			}

			out[i] = S;
		}
	}
}

void b3SpringSolver::Solve(b3DenseVec3& x, b3DenseVec3& f, u32& iterations, 
	const b3SparseMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const
{
	// P = diag(A)
	b3DiagMat33 inv_P(m_massCount);
	A.AssembleDiagonal(inv_P);

	// Invert
	for (u32 i = 0; i < m_massCount; ++i)
	{
		b3Mat33& D = inv_P[i];
		
		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(D.x, D.y, D.z) > B3_EPSILON);

		B3_ASSERT(D.x.x != 0.0f);
		float32 xx = 1.0f / D.x.x;

		B3_ASSERT(D.y.y != 0.0f);
		float32 yy = 1.0f / D.y.y;
		
		B3_ASSERT(D.z.z != 0.0f);
		float32 zz = 1.0f / D.z.z;

		D = b3Diagonal(xx, yy, zz);
	}

	// I
	b3DiagMat33 I(m_massCount);
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
	B3_ASSERT(b3IsValid(deltaNew));

	// Tolerance.
	// This is the main stopping criteria.
	// [0, 1]
	const float32 epsilon = 10.0f * B3_EPSILON;

	// Maximum number of iterations.
	const u32 maxIters = 1000;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < maxIters && deltaNew > epsilon * epsilon * b_delta)
	{
		// s = S(A * p)
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
		B3_ASSERT(b3IsValid(deltaNew));

		// beta = deltaNew / deltaOld
		float32 beta = deltaNew / deltaOld;

		// p = S * (h + beta * p)
		p = S * (h + beta * p);

		++iter;
	}

	iterations = iter;

	// Residual
	f = A * x - b;
}