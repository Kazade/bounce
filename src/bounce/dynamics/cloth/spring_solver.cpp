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
#include <bounce/dynamics/cloth/sparse_mat33.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Conjugate Gradient method.
// This work is based on the paper "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Enable preconditioning. It can be slow, depending on 
// how the preconditioning matrix is computed, but it can help
// to increase convergence.
bool b3_enablePrecontitioning = false;

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
	m_types = m_cloth->m_types;
	m_massCount = m_cloth->m_massCount;

	m_contacts = m_cloth->m_contacts;

	m_springs = m_cloth->m_springs;
	m_springCount = m_cloth->m_springCount;
}

b3SpringSolver::~b3SpringSolver()
{

}

void b3SpringSolver::Solve(b3DenseVec3& f)
{
	//
	m_Jx = (b3Mat33*)m_allocator->Allocate(m_springCount * sizeof(b3Mat33));
	m_Jv = (b3Mat33*)m_allocator->Allocate(m_springCount * sizeof(b3Mat33));

	// Apply spring forces. Also, store their unique derivatives.
	ApplySpringForces();

	// Integrate

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	// Allocate matrix memory for the worst case.
	u32 nzCount = m_massCount * m_massCount;
	b3Mat33* nzElements = (b3Mat33*)m_allocator->Allocate(nzCount * sizeof(b3Mat33));
	u32* nzColumns = (u32*)m_allocator->Allocate(nzCount * sizeof(u32));
	u32* rowPtrs = (u32*)m_allocator->Allocate((m_massCount + 1) * sizeof(u32));

	{
		b3SparseMat33 A(m_massCount, m_massCount, nzCount, nzElements, rowPtrs, nzColumns);

		//
		b3DenseVec3 b(m_massCount);
		
		//
		Compute_A_b(A, b);

		// x
		b3DenseVec3 x(m_massCount);

		if (b3_enablePrecontitioning)
		{
			Solve_MPCG(x, A, f, m_iterations, b);
		}
		else
		{
			Solve_MCG(x, A, f, m_iterations, b);
		}

		// Update state
		for (u32 i = 0; i < m_massCount; ++i)
		{
			m_v[i] += x[i];

			// dx = h * (v0 + dv) + y = h * v1 + y
			m_x[i] += m_h * m_v[i] + m_y[i];
		}
	}

	m_allocator->Free(rowPtrs);
	m_allocator->Free(nzColumns);
	m_allocator->Free(nzElements);

	m_allocator->Free(m_Jv);
	m_allocator->Free(m_Jx);
	m_Jv = nullptr;
	m_Jx = nullptr;
}

#define B3_INDEX(i, j, size) (i + j * size)

static void b3SetZero(b3Vec3* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

static void b3SetZero(b3Mat33* out, u32 size)
{
	for (u32 i = 0; i < size * size; ++i)
	{
		out[i].SetZero();
	}
}

static void b3SetZero_Jacobian(b3Mat33* out, u32 springCount)
{
	for (u32 i = 0; i < springCount; ++i)
	{
		out[i].SetZero();
	}
}

// J = dfdx or dvdx
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
	b3SetZero_Jacobian(m_Jx, m_springCount);
	b3SetZero_Jacobian(m_Jv, m_springCount);

	// Compute forces and Jacobians
	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* S = m_springs + i;

		b3SpringType type = S->type;
		u32 i1 = S->i1;
		u32 i2 = S->i2;
		float32 L0 = S->L0;
		float32 ks = S->ks;
		float32 kd = S->kd;

		b3Vec3 x1 = m_x[i1];
		b3Vec3 v1 = m_v[i1];

		b3Vec3 x2 = m_x[i2];
		b3Vec3 v2 = m_v[i2];

		// Compute strech forces
		b3Vec3 dx = x1 - x2;
		float32 L = b3Length(dx);

		B3_ASSERT(L > 0.0f);

		b3Vec3 sf1 = -ks * (1.0f - L0 / L) * dx;
		b3Vec3 sf2 = -sf1;

		m_f[i1] += sf1;
		m_f[i2] += sf2;

		// C * n = 1 - L0 / L * dx
		const b3Mat33 I = b3Mat33_identity;

		float32 L3 = L * L * L;

		b3Mat33 Jx11 = -ks * ((1.0f - L0 / L) * I + (L0 / L3) * b3Outer(dx, dx));

		m_Jx[i] = Jx11;

		// Compute damping forces
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
	bool isZeroX = b3Dot(A.x, A.x) <= B3_EPSILON * B3_EPSILON;
	bool isZeroY = b3Dot(A.y, A.y) <= B3_EPSILON * B3_EPSILON;
	bool isZeroZ = b3Dot(A.z, A.z) <= B3_EPSILON * B3_EPSILON;

	return isZeroX * isZeroY * isZeroZ;
}

void b3SpringSolver::Compute_A_b(b3SparseMat33& SA, b3DenseVec3& b) const
{
	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)m_allocator->Allocate(m_massCount * m_massCount * sizeof(b3Mat33));
	b3SetZero(dfdx, m_massCount);

	b3Mat33* dfdv = (b3Mat33*)m_allocator->Allocate(m_massCount * m_massCount * sizeof(b3Mat33));
	b3SetZero(dfdv, m_massCount);

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
	b3SetZero(A, m_massCount);

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

#if 0
	for (u32 i = 0; i < m_massCount * m_massCount; ++i)
	{
		b3Mat33 a = A[i];
		if (b3IsZero(a) == false)
		{
			++nzCount;
		}
}
#endif

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
	// b = h * (f0 + h * Jx_v + Jx_y )
	
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

// This outputs the desired acceleration of the masses in the constrained 
// directions.
static void b3Compute_z(b3DenseVec3& out,
	u32 massCount, const b3MassType* types, const b3MassContact* contacts)
{
	out.SetZero();
}

// Maintains invariants inside the MCG solver.
static void b3Filter(b3DenseVec3& out,
	const b3DenseVec3& v, u32 massCount, const b3MassType* types, const b3MassContact* contacts)
{
	for (u32 i = 0; i < massCount; ++i)
	{
		switch (types[i])
		{
		case e_staticMass:
		{
			out[i].SetZero();
			break;
		}
		case e_dynamicMass:
		{
			if (contacts[i].lockN == true)
			{
				b3Vec3 n = contacts[i].n;

				b3Mat33 S = b3Mat33_identity - b3Outer(n, n);

				if (contacts[i].lockT1 == true)
				{
					b3Vec3 t1 = contacts[i].t1;

					S -= b3Outer(t1, t1);
				}

				if (contacts[i].lockT2 == true)
				{
					b3Vec3 t2 = contacts[i].t2;

					S -= b3Outer(t2, t2);
				}

				out[i] = S * v[i];
				break;
			}

			out[i] = v[i];
			break;
		}
		default:
		{
			B3_ASSERT(false);
			break;
		}
		}
	}
}

void b3SpringSolver::Solve_MCG(b3DenseVec3& dv, const b3SparseMat33& A, b3DenseVec3& e, u32& iterations, const b3DenseVec3& b) const
{
	// dv = z
	b3Compute_z(dv, m_massCount, m_types, m_contacts);

	// Adv = A * dv 
	b3DenseVec3 Adv(m_massCount);
	A.Mul(Adv, dv);

	// r = filter(b - Adv)
	b3DenseVec3 r(m_massCount);
	b3Sub(r, b, Adv);
	b3Filter(r, r, m_massCount, m_types, m_contacts);

	// c = r
	b3DenseVec3 c = r;

	// eps0 = dot(r, r)
	float32 eps0 = b3Dot(r, r);

	// epsNew = dot(r, r)
	float32 epsNew = eps0;

	// [0, 1]
	const float32 kTol = 10.0f * B3_EPSILON;

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 10 * 10;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters && epsNew > kTol * kTol * eps0)
	{
		// q = filter(A * c) 
		b3DenseVec3 q(m_massCount);
		A.Mul(q, c);
		b3Filter(q, q, m_massCount, m_types, m_contacts);

		// alpha = epsNew / dot(c, q)
		float32 alpha_den = b3Dot(c, q);
		float32 alpha = epsNew / alpha_den;

		// dv = dv + alpha * c
		b3DenseVec3 alpha_c(m_massCount);
		b3Mul(alpha_c, alpha, c);
		b3Add(dv, dv, alpha_c);

		// r = r - alpha * q 
		b3DenseVec3 alpha_q(m_massCount);
		b3Mul(alpha_q, alpha, q);
		b3Sub(r, r, alpha_q);

		// epsOld = epsNew
		float32 epsOld = epsNew;

		// epsNew = dot(r, r) 
		epsNew = b3Dot(r, r);

		float32 beta = epsNew / epsOld;

		// c = filter(r + beta * c) 
		b3DenseVec3 beta_c(m_massCount);
		b3Mul(beta_c, beta, c);
		b3Add(c, r, beta_c);
		b3Filter(c, c, m_massCount, m_types, m_contacts);

		++iter;
	}

	iterations = iter;

	// f = A * dv - b
	A.Mul(e, dv);
	b3Sub(e, e, b);
}

// Sylvester's Criterion
static bool b3IsPD(const b3Mat33* diagA, u32 n)
{
	// Loop over the principal elements
	for (u32 i = 0; i < n; ++i)
	{
		b3Mat33 a = diagA[i];
		
		float32 D = b3Det(a.x, a.y, a.z);

		const float32 kTol = 0.0f;

		if (D <= kTol)
		{
			return false;
		}
	}

	return true;
}

void b3SpringSolver::Solve_MPCG(b3DenseVec3& dv, const b3SparseMat33& A, b3DenseVec3& e, u32& iterations, const b3DenseVec3& b) const
{
	b3DenseVec3 r(m_massCount);
	
	b3DenseVec3 c(m_massCount);
	
	b3DenseVec3 s(m_massCount);

	b3DenseVec3 inv_P(m_massCount);

	// dv = z
	b3Compute_z(dv, m_massCount, m_types, m_contacts);

	// P = diag(A)^-1
	b3DenseVec3 P(m_massCount);

	// diag(A)
	b3Mat33* diagA = (b3Mat33*)m_allocator->Allocate(m_massCount * sizeof(b3Mat33));
	A.AssembleDiagonal(diagA);

	// Compute P, P^-1
	bool isPD = true;

	for (u32 i = 0; i < m_massCount; ++i)
	{
		b3Mat33 D = diagA[i];

		if (b3Det(D.x, D.y, D.z) <= 3.0f * B3_EPSILON)
		{
			isPD = false;
		}

		B3_ASSERT(D[0][0] != 0.0f);
		B3_ASSERT(D[1][1] != 0.0f);
		B3_ASSERT(D[2][2] != 0.0f);

		P[i] = b3Vec3(1.0f / D[0][0], 1.0f / D[1][1], 1.0f / D[2][2]);
		inv_P[i] = b3Vec3(D[0][0], D[1][1], D[2][2]);
	}

	m_allocator->Free(diagA);
	
	// eps0 = dot( filter(b), P * filter(b) )
	b3DenseVec3 filter_b(m_massCount);
	b3Filter(filter_b, b, m_massCount, m_types, m_contacts);

	b3DenseVec3 P_filter_b(m_massCount);
	for (u32 i = 0; i < m_massCount; ++i)
	{
		P_filter_b[i][0] = P[i][0] * filter_b[i][0];
		P_filter_b[i][1] = P[i][1] * filter_b[i][1];
		P_filter_b[i][2] = P[i][2] * filter_b[i][2];
	}

	float32 eps0 = b3Dot(filter_b, P_filter_b);
	
	// r = filter(b - Adv)

	// Adv = A * dv
	b3DenseVec3 Adv(m_massCount);
	A.Mul(Adv, dv);
	b3Sub(r, b, Adv);
	b3Filter(r, r, m_massCount, m_types, m_contacts);

	// c = filter(P^-1 * r)
	for (u32 i = 0; i < m_massCount; ++i)
	{
		c[i][0] = inv_P[i][0] * r[i][0];
		c[i][1] = inv_P[i][1] * r[i][1];
		c[i][2] = inv_P[i][2] * r[i][2];
	}
	b3Filter(c, c, m_massCount, m_types, m_contacts);
	
	// epsNew = dot(r, c)
	float32 epsNew = b3Dot(r, c);

	// [0, 1]
	const float32 kTol = 10.0f * B3_EPSILON;

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 10 * 10;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters && epsNew > kTol * kTol * eps0)
	{
		// q = filter(A * c)
		b3DenseVec3 q(m_massCount);
		A.Mul(q, c);
		b3Filter(q, q, m_massCount, m_types, m_contacts);

		// alpha = epsNew / dot(c, q)
		float32 alpha = epsNew / b3Dot(c, q);

		// x = x + alpha * c
		for (u32 i = 0; i < m_massCount; ++i)
		{
			dv[i] = dv[i] + alpha * c[i];
		}

		// r = r - alpha * q
		for (u32 i = 0; i < m_massCount; ++i)
		{
			r[i] = r[i] - alpha * q[i];
		}

		// s = inv_P * r
		for (u32 i = 0; i < m_massCount; ++i)
		{
			s[i][0] = inv_P[i][0] * r[i][0];
			s[i][1] = inv_P[i][1] * r[i][1];
			s[i][2] = inv_P[i][2] * r[i][2];
		}

		// epsOld = epsNew
		float32 epsOld = epsNew;

		// epsNew = dot(r, s)
		epsNew = b3Dot(r, s);

		// beta = epsNew / epsOld
		float32 beta = epsNew / epsOld;

		// c = filter(s + beta * c)
		for (u32 i = 0; i < m_massCount; ++i)
		{
			c[i] = s[i] + beta * c[i];
		}
		b3Filter(c, c, m_massCount, m_types, m_contacts);

		++iter;
	}

	iterations = iter;

	// Residual error
	// f = A * x - b
	A.Mul(e, dv);
	b3Sub(e, e, b);
}