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

void b3SpringSolver::Solve(b3Vec3* f)
{
	u32 size = m_massCount;
	b3MassType* types = m_types;
	u32 spring_size = m_springCount;

	m_Jx = (b3Mat33*)m_allocator->Allocate(spring_size * sizeof(b3Mat33));
	m_Jv = (b3Mat33*)m_allocator->Allocate(spring_size * sizeof(b3Mat33));

	// Compute and apply spring forces, store their unique derivatives.
	InitializeSpringForces();

	// Integrate

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 
	
	//
	b3Vec3* b = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	Compute_b(b);
	
	//
	b3Vec3* x = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	// Solve Ax = b
	if (b3_enablePrecontitioning)
	{
		Solve_MPCG(x, f, m_iterations, b);
	}
	else
	{
		Solve_MCG(x, f, m_iterations, b);
	}

	// Update state
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_v[i] += x[i];

		// dx = h * (v0 + dv) + y = h * v1 + y
		m_x[i] += m_h * m_v[i] + m_y[i];
	}

	m_allocator->Free(x);
	m_allocator->Free(b);

	m_allocator->Free(m_Jv);
	m_allocator->Free(m_Jx);
	m_Jv = nullptr;
	m_Jx = nullptr;
}

// This outputs the desired acceleration of the masses in the constrained 
// directions.
static B3_FORCE_INLINE void b3Compute_z(b3Vec3* out, 
	u32 size, const b3MassType* types, const b3MassContact* contacts)
{
	for (u32 i = 0; i < size; ++i)
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
			if (contacts[i].lockOnSurface)
			{
				out[i].SetZero();
				break;
			}

			out[i].SetZero();
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

static B3_FORCE_INLINE void b3Filter(b3Vec3* out, 
	const b3Vec3* v, u32 size, const b3MassType* types, const b3MassContact* contacts)
{
	for (u32 i = 0; i < size; ++i)
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
			if (contacts[i].lockOnSurface)
			{				
				b3Vec3 n = contacts[i].n;

				b3Mat33 S = b3Mat33_identity - b3Outer(n, n);

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

static B3_FORCE_INLINE void b3SetZero(b3Vec3* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

static B3_FORCE_INLINE void b3Copy(b3Vec3* out, const b3Vec3* v, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i] = v[i];
	}
}

static B3_FORCE_INLINE void b3Add(b3Vec3* out, const b3Vec3* a, const b3Vec3* b, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i] = a[i] + b[i];
	}
}

static B3_FORCE_INLINE void b3Sub(b3Vec3* out, const b3Vec3* a, const b3Vec3* b, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i] = a[i] - b[i];
	}
}

static B3_FORCE_INLINE float32 b3Dot(const b3Vec3* a, const b3Vec3* b, u32 size)
{
	float32 result = 0.0f;
	for (u32 i = 0; i < size; ++i)
	{
		result += b3Dot(a[i], b[i]);
	}
	return result;
}

#define B3_INDEX(i, j, size) (i + j * size)

static B3_FORCE_INLINE void b3SetZero(b3Mat33* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		for (u32 j = 0; j < size; ++j)
		{
			out[B3_INDEX(i, j, size)].SetZero();
		}
	}
}

static B3_FORCE_INLINE void b3Mul(b3Vec3* out, const b3Mat33* M, const b3Vec3* v, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();

		for (u32 j = 0; j < size; ++j)
		{
			out[i] += M[B3_INDEX(i, j, size)] * v[j];
		}
	}
}

// J = dfdx or dvdx
static B3_FORCE_INLINE void b3Mul_Jacobian(b3Vec3* out, const b3Vec3* v, u32 mass_size,
	const b3Mat33* J_ii, const b3Spring* springs, u32 spring_size)
{
	b3SetZero(out, mass_size);

	for (u32 i = 0; i < spring_size; ++i)
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

static B3_FORCE_INLINE void b3SetZero_Jacobian(b3Mat33* out, u32 size)
{
	for (u32 i = 0; i < size; ++i)
	{
		out[i].SetZero();
	}
}

// A = M - h * dfdv - h * h * dfdx
// A * v = (M - h * dfdv - h * h * dfdx) * v = M * v + (-h * dfdv * v) + (-h * h * dfdx * v) 
static B3_FORCE_INLINE void b3Mul_A(b3Vec3* out, const b3Vec3* v, u32 mass_size,
	b3StackAllocator* allocator,
	const float32* inv_m, float32 h, const b3Mat33* Jx, const b3Mat33* Jv, const b3Spring* springs, u32 spring_size)
{
	// v1 = M * v
	b3Vec3* v1 = (b3Vec3*)allocator->Allocate(mass_size * sizeof(b3Vec3));
	for (u32 i = 0; i < mass_size; ++i)
	{
		float32 m = inv_m[i] != 0.0f ? 1.0f / inv_m[i] : 0.0f;
		v1[i] = m * v[i];
	}

	// v2 = (-h * dfdv * v)
	b3Vec3* v2 = (b3Vec3*)allocator->Allocate(mass_size * sizeof(b3Vec3));
	b3Mul_Jacobian(v2, v, mass_size, Jv, springs, spring_size);
	for (u32 i = 0; i < mass_size; ++i)
	{
		v2[i] *= -h;
	}

	// v3 = (-h * h * dfdx * v)
	b3Vec3* v3 = (b3Vec3*)allocator->Allocate(mass_size * sizeof(b3Vec3));
	b3Mul_Jacobian(v3, v, mass_size, Jx, springs, spring_size);
	for (u32 i = 0; i < mass_size; ++i)
	{
		v3[i] *= -h * h;
	}

	// v = v1 + v2 + v3
	for (u32 i = 0; i < mass_size; ++i)
	{
		out[i] = v1[i] + v2[i] + v3[i];
	}

	allocator->Free(v3);
	allocator->Free(v2);
	allocator->Free(v1);
}

void b3SpringSolver::InitializeSpringForces()
{
	u32 size = m_massCount;
	u32 spring_size = m_springCount;

	// Zero Jacobians
	b3SetZero_Jacobian(m_Jx, spring_size);
	b3SetZero_Jacobian(m_Jv, spring_size);

	// Compute forces and Jacobians
	for (u32 i = 0; i < spring_size; ++i)
	{
		b3Spring* S = m_springs + i;

		b3Vec3 x1 = m_x[S->i1];
		b3Vec3 v1 = m_v[S->i1];

		b3Vec3 x2 = m_x[S->i2];
		b3Vec3 v2 = m_v[S->i2];

		// Strech
		b3Vec3 dx = x2 - x1;
		float32 L = b3Length(dx);
		b3Vec3 n = dx;
		if (L > 0.0f)
		{
			n /= L;
		}

		float32 C = L - S->L0;

		// Compute streching forces
		b3Vec3 sf1 = -S->ks * C * -n;
		b3Vec3 sf2 = -sf1;

		m_f[S->i1] += sf1;
		m_f[S->i2] += sf2;

		// Compute damping forces
		b3Vec3 dv = v2 - v1;

		b3Vec3 df1 = -S->kd * -dv;
		b3Vec3 df2 = -df1;

		m_f[S->i1] += df1;
		m_f[S->i2] += df2;

		b3Mat33 I = b3Mat33_identity;

		// Compute Jx11	
		float32 inv_L = L > 0.0f ? 1.0f / L : 0.0f;

		float32 L2 = L * L;
		float32 inv_L2 = L2 > 0.0f ? 1.0f / L2 : 0.0f;

		// Hessian
		// del^2_C / del_x
		b3Mat33 H_11 = inv_L * I + inv_L2 * b3Outer(dx, -n);

		// del_C / del_x * del_C / del_x^T
		b3Mat33 JJ_11 = b3Outer(-n, -n);

		b3Mat33 Jx11 = -S->ks * (C * H_11 + JJ_11);
		m_Jx[i] = Jx11;

		// Compute Jv11
		b3Mat33 Jv11 = -S->kd * I;
		m_Jv[i] = Jv11;
	}
}

void b3SpringSolver::Compute_b(b3Vec3* b) const
{
	float32 h = m_h;
	u32 size = m_massCount;

	// Jx_v = dfdx * v
	b3Vec3* Jx_v = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_v, m_v, size, m_Jx, m_springs, m_springCount);

	// Jx_y = dfdx * y
	b3Vec3* Jx_y = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	b3Mul_Jacobian(Jx_y, m_y, size, m_Jx, m_springs, m_springCount);

	// b = h * (f0 + h * Jx_v + Jx_y )
	for (u32 i = 0; i < size; ++i)
	{
		b[i] = h * (m_f[i] + h * Jx_v[i] + Jx_y[i]);
	}

	m_allocator->Free(Jx_y);
	m_allocator->Free(Jx_v);
}

void b3SpringSolver::Solve_MCG(b3Vec3* dv, b3Vec3* e, u32& iterations, const b3Vec3* b) const
{
	// dv = z
	b3Compute_z(dv, m_massCount, m_types, m_contacts);

	// r = filter(b - Adv)
	b3Vec3* r = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));

	// Adv = A * dv 
	b3Vec3* Adv = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));

	b3Mul_A(Adv, dv, m_massCount, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, m_springCount);
	b3Sub(r, b, Adv, m_massCount);
	b3Filter(r, r, m_massCount, m_types, m_contacts);

	m_allocator->Free(Adv);

	// c = r
	b3Vec3* c = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));
	b3Copy(c, r, m_massCount);

	// eps0 = dot(f, f)
	float32 eps0 = b3Dot(r, r, m_massCount);

	// epsNew = dot(r, r)
	float32 epsNew = eps0;

	// [0, 1]
	const float32 kTol = 0.25f;

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 100;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters && epsNew > kTol * kTol * eps0)
	{
		// q = filter(A * c) 
		b3Vec3* q = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));
		b3Mul_A(q, c, m_massCount, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, m_springCount);
		b3Filter(q, q, m_massCount, m_types, m_contacts);

		// alpha = epsNew / dot(c, q)
		float32 alpha_den = b3Dot(c, q, m_massCount);
		float32 alpha = epsNew / alpha_den;

		// dv = dv + alpha * c
		for (u32 i = 0; i < m_massCount; ++i)
		{
			dv[i] = dv[i] + alpha * c[i];
		}

		// r = r - alpha * q 
		for (u32 i = 0; i < m_massCount; ++i)
		{
			r[i] = r[i] - alpha * q[i];
		}

		m_allocator->Free(q);

		// epsOld = epsNew
		float32 epsOld = epsNew;

		// epsNew = dot(r, r) 
		epsNew = b3Dot(r, r, m_massCount);

		float32 beta = epsNew / epsOld;

		// c = filter(r + beta * c) 
		for (u32 i = 0; i < m_massCount; ++i)
		{
			c[i] = r[i] + beta * c[i];
		}
		b3Filter(c, c, m_massCount, m_types, m_contacts);

		++iter;
	}

	m_allocator->Free(c);
	m_allocator->Free(r);

	iterations = iter;

	// f = A * dv - b
	b3Mul_A(e, dv, m_massCount, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, m_springCount);
	b3Sub(e, e, b, m_massCount);
}

static void B3_FORCE_INLINE b3Make_A(b3Mat33* A,
	const b3Mat33* Jx, const b3Mat33* Jv, u32 size,
	b3StackAllocator* allocator, float32 h, float32* inv_m,
	const b3Spring* springs, u32 spring_size)
{
	// A = M - h * dfdv - h * h * dfdx

	// A = 0
	b3SetZero(A, size);

	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)allocator->Allocate(size * size * sizeof(b3Mat33));
	b3SetZero(dfdx, size);

	b3Mat33* dfdv = (b3Mat33*)allocator->Allocate(size * size * sizeof(b3Mat33));
	b3SetZero(dfdv, size);

	for (u32 i = 0; i < spring_size; ++i)
	{
		const b3Spring* S = springs + i;

		b3Mat33 Jx11 = Jx[i];
		b3Mat33 Jx12 = -Jx11;
		b3Mat33 Jx21 = Jx12;
		b3Mat33 Jx22 = Jx11;

		dfdx[B3_INDEX(S->i1, S->i1, size)] += Jx11;
		dfdx[B3_INDEX(S->i1, S->i2, size)] += Jx12;
		dfdx[B3_INDEX(S->i2, S->i1, size)] += Jx21;
		dfdx[B3_INDEX(S->i2, S->i2, size)] += Jx22;

		b3Mat33 Jv11 = Jv[i];
		b3Mat33 Jv12 = -Jv11;
		b3Mat33 Jv21 = Jv12;
		b3Mat33 Jv22 = Jv11;

		dfdv[B3_INDEX(S->i1, S->i1, size)] += Jv11;
		dfdv[B3_INDEX(S->i1, S->i2, size)] += Jv12;
		dfdv[B3_INDEX(S->i2, S->i1, size)] += Jv21;
		dfdv[B3_INDEX(S->i2, S->i2, size)] += Jv22;
	}

	// A += M
	for (u32 i = 0; i < size; ++i)
	{
		B3_ASSERT(inv_m[i] != 0.0f);

		float32 m = 1.0f / inv_m[i];

		A[B3_INDEX(i, i, size)] += b3Diagonal(m);
	}

	// A += - h * dfdv - h * h * dfdx
	for (u32 i = 0; i < size; ++i)
	{
		for (u32 j = 0; j < size; ++j)
		{
			A[B3_INDEX(i, j, size)] += (-h * dfdv[B3_INDEX(i, j, size)]) + (-h * h * dfdx[B3_INDEX(i, j, size)]);
		}
	}

	allocator->Free(dfdv);
	allocator->Free(dfdx);
}

void b3SpringSolver::Solve_MPCG(b3Vec3* dv, b3Vec3* e, u32& iterations, const b3Vec3* b) const
{
	u32 size = m_massCount;
	b3MassType* types = m_types;
	u32 spring_size = m_springCount;

	b3Vec3* r = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* c = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* s = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* inv_P = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	// dv = z
	b3Compute_z(dv, size, types, m_contacts);

	// P = diag(A)^-1
	b3Vec3* P = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	// A = M - h * dfdv - h * h * dfdx
	b3Mat33* A = (b3Mat33*)m_allocator->Allocate(size * size * sizeof(b3Mat33));
	b3Make_A(A, m_Jx, m_Jv, size, m_allocator, m_h, m_inv_m, m_springs, m_springCount);

	// Compute P, P^-1
	// @todo Optimize so we don't need to compute A.
	for (u32 i = 0; i < size; ++i)
	{
		b3Mat33 D = A[B3_INDEX(i, i, size)];

		B3_ASSERT(D[0][0] != 0.0f);
		B3_ASSERT(D[1][1] != 0.0f);
		B3_ASSERT(D[2][2] != 0.0f);

		P[i] = b3Vec3(1.0f / D[0][0], 1.0f / D[1][1], 1.0f / D[2][2]);
		inv_P[i] = b3Vec3(D[0][0], D[1][1], D[2][2]);
	}

	m_allocator->Free(A);

	// eps0 = dot( filter(b), P * filter(b) )
	b3Vec3* filter_b = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	b3Filter(filter_b, b, size, types, m_contacts);

	b3Vec3* P_filter_b = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	for (u32 i = 0; i < size; ++i)
	{
		P_filter_b[i][0] = P[i][0] * filter_b[i][0];
		P_filter_b[i][1] = P[i][1] * filter_b[i][1];
		P_filter_b[i][2] = P[i][2] * filter_b[i][2];
	}

	float32 eps0 = b3Dot(filter_b, P_filter_b, size);

	m_allocator->Free(P_filter_b);
	m_allocator->Free(filter_b);
	m_allocator->Free(P);

	// r = filter(b - Adv)

	// Adv = A * dv 
	b3Vec3* Adv = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	b3Mul_A(Adv, dv, size, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, m_springCount);

	b3Sub(r, b, Adv, size);

	m_allocator->Free(Adv);

	b3Filter(r, r, size, types, m_contacts);

	// c = filter(P^-1 * r)
	for (u32 i = 0; i < m_massCount; ++i)
	{
		c[i][0] = inv_P[i][0] * r[i][0];
		c[i][1] = inv_P[i][1] * r[i][1];
		c[i][2] = inv_P[i][2] * r[i][2];
	}
	b3Filter(c, c, size, types, m_contacts);

	// epsNew = dot(r, c)
	float32 epsNew = b3Dot(r, c, size);

	// [0, 1]
	const float32 kTol = 0.25f;

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 100;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters && epsNew > kTol * kTol * eps0)
	{
		// q = filter(A * c) 
		b3Vec3* q = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

		b3Mul_A(q, c, size, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, m_springCount);
		b3Filter(q, q, size, types, m_contacts);

		// alpha = epsNew / dot(c, q)
		float32 alpha = epsNew / b3Dot(c, q, size);

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

		m_allocator->Free(q);

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
		epsNew = b3Dot(r, s, size);

		// beta = epsNew / epsOld
		float32 beta = epsNew / epsOld;

		// c = filter(s + beta * c) 
		for (u32 i = 0; i < m_massCount; ++i)
		{
			c[i] = s[i] + beta * c[i];
		}
		b3Filter(c, c, size, types, m_contacts);

		++iter;
	}

	m_allocator->Free(inv_P);
	m_allocator->Free(s);
	m_allocator->Free(c);
	m_allocator->Free(r);

	iterations = iter;

	// Residual error
	// f = A * x - b
	b3Mul_A(e, dv, size, m_allocator, m_inv_m, m_h, m_Jx, m_Jv, m_springs, spring_size);
	b3Sub(e, e, b, size);
}