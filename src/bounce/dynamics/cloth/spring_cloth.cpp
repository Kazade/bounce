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

#include <bounce/dynamics/cloth/spring_cloth.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/common/memory/stack_allocator.h>

// Here, we solve Ax = b using the Modified Conjugate Gradient method.
// This work is based on the paper "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

b3SpringCloth::b3SpringCloth()
{
	m_allocator = nullptr;

	m_mesh = nullptr;

	m_gravity.SetZero();

	m_x = nullptr;
	m_v = nullptr;
	m_f = nullptr;
	m_y = nullptr;
	m_inv_m = nullptr;
	m_massTypes = nullptr;
	m_collisions = nullptr;
	m_massCount = 0;
	
	m_springs = nullptr;
	m_springCount = 0; 
	
	m_r = 0.0f;

	m_sphereCount = 0;

	m_step.iterations = 0;
}

b3SpringCloth::~b3SpringCloth()
{
	b3Free(m_x);
	b3Free(m_v);
	b3Free(m_f);
	b3Free(m_inv_m);
	b3Free(m_y);
	b3Free(m_massTypes);
	b3Free(m_collisions);
	b3Free(m_springs);
}

void b3SpringCloth::Initialize(const b3SpringClothDef& def)
{
	B3_ASSERT(def.allocator);
	B3_ASSERT(def.mesh);

	m_allocator = def.allocator;
	m_mesh = def.mesh;
	m_gravity = def.gravity;

	m_r = def.r;
	
	const b3Mesh* m = m_mesh;

	m_massCount = m->vertexCount;
	m_x = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_v = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_f = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_inv_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_y = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_massTypes = (b3MassType*)b3Alloc(m_massCount * sizeof(b3MassType));
	m_collisions = (b3MassCollision*)b3Alloc(m_massCount * sizeof(b3MassCollision));

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		m_collisions[i].active = false;
		m_x[i] = m->vertices[i];
		m_v[i].SetZero();
		m_f[i].SetZero();
		m_inv_m[i] = 0.0f;
		m_y[i].SetZero();
		m_massTypes[i] = e_staticMass;
	}

	// Initialize mass
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Vec3 p1 = m->vertices[t->v1];
		b3Vec3 p2 = m->vertices[t->v2];
		b3Vec3 p3 = m->vertices[t->v3];

		float32 area = b3Area(p1, p2, p3);
		float32 mass = def.density * area;

		const float32 inv3 = 1.0f / 3.0f;

		m_inv_m[t->v1] += inv3 * mass;
		m_inv_m[t->v2] += inv3 * mass;
		m_inv_m[t->v3] += inv3 * mass;
	}
	
	// Invert
	for (u32 i = 0; i < m_massCount; ++i)
	{
		if (m_inv_m[i] > 0.0f)
		{
			m_inv_m[i] = 1.0f / m_inv_m[i];
			m_massTypes[i] = e_dynamicMass;
		}
	}

	// Initialize springs
	m_springs = (b3Spring*)b3Alloc(3 * m->triangleCount * sizeof(b3Spring));
	
	// Streching
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;
		
		u32 is[3] = { t->v1, t->v2, t->v3 };
		for (u32 j = 0; j < 3; ++j)
		{
			u32 k = j + 1 < 3 ? j + 1 : 0;

			u32 v1 = is[j];
			u32 v2 = is[k];

			b3Vec3 p1 = m->vertices[v1];
			b3Vec3 p2 = m->vertices[v2];

			// Skip duplicated spring
			bool found = false;
			for (u32 s = 0; s < m_springCount; ++s)
			{
				if ((m_springs[s].i1 == v1 && m_springs[s].i2 == v2) || (m_springs[s].i1 == v2 && m_springs[s].i2 == v1))
				{
					found = true;
					break;
				}
			}

			if (found == false)
			{
				b3Spring* S = m_springs + m_springCount;
				S->i1 = v1;
				S->i2 = v2;
				S->L0 = b3Distance(p1, p2);
				S->ks = def.ks;
				S->kd = def.kd;
				++m_springCount;
			}
		}
	}
}

b3Sphere* b3SpringCloth::CreateSphere(const b3Vec3& center, float32 radius)
{
	B3_ASSERT(m_sphereCount < B3_CLOTH_SPHERE_CAPACITY);
	
	if (m_sphereCount == B3_CLOTH_SPHERE_CAPACITY)
	{
		return nullptr;
	}

	b3Sphere* sphere = m_spheres + m_sphereCount;
	sphere->vertex = center;
	sphere->radius = radius;
	++m_sphereCount;
	return sphere;
}

static B3_FORCE_INLINE void b3Make_z(b3Vec3* out, u32 size,
	const b3MassType* types, const b3MassCollision* collisions)
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
			if (collisions[i].active)
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

static B3_FORCE_INLINE void b3Filter(b3Vec3* out, const b3Vec3* v, u32 size, 
	const b3MassType* types, const b3MassCollision* collisions)
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
			if (collisions[i].active)
			{
				b3Vec3 n = collisions[i].n;

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
			out[i] += M[ B3_INDEX(i, j, size) ] * v[j];
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

void b3SpringCloth::UpdateCollisions() const
{
	// Compute cloth-solid collision position alteration
	for (u32 i = 0; i < m_massCount; ++i)
	{
		// Clear flag
		m_collisions[i].active = false;

		b3Vec3 c1 = m_x[i];
		float32 r1 = m_r;

		// Only solve the deepest penetrations
		float32 bestSeparation = B3_MAX_FLOAT;
		u32 bestIndex = ~0;

		for (u32 j = 0; j < m_sphereCount; ++j)
		{
			const b3Sphere* sphere = m_spheres + j;
			b3Vec3 c2 = sphere->vertex;
			float32 r2 = sphere->radius;

			b3Vec3 d = c2 - c1;
			float32 dd = b3Dot(d, d);
			float32 totalRadius = r1 + r2;
			if (dd > totalRadius * totalRadius)
			{
				continue;
			}

			float32 distance = b3Length(d);
			float32 separation = distance - totalRadius;

			if (separation < bestSeparation)
			{
				bestSeparation = separation;
				bestIndex = j;
			}
		}

		if (bestIndex != ~0)
		{
			const b3Sphere* sphere = m_spheres + bestIndex;
			b3Vec3 c2 = sphere->vertex;
			float32 r2 = sphere->radius;

			float32 totalRadius = r1 + r2;

			b3Vec3 d = c2 - c1;
			float32 distance = b3Length(d);
			float32 separation = distance - totalRadius;

			b3Vec3 n(0.0f, 1.0f, 0.0f);
			if (distance > B3_EPSILON)
			{
				n = d / distance;
			}

			// Avoid large corrections
			const float32 kMaxCorrection = 0.75f;
			separation = b3Clamp(separation, -kMaxCorrection, 0.0f);

			b3Vec3 dx1 = separation * n;

			// Add position alteration
			m_y[i] += dx1;

			m_collisions[i].active = true;
			m_collisions[i].j = bestIndex;
			m_collisions[i].s = separation;
			m_collisions[i].n = n;
		}
	}
}

void b3SpringCloth::Step(float32 h)
{
	if (h == 0.0f)
	{
		return;
	}

	// Detect and store collisions
	UpdateCollisions();

	u32 size = m_massCount;
	b3MassType* types = m_massTypes;
	u32 spring_size = m_springCount;

	// Add gravity
	for (u32 i = 0; i < size; ++i)
	{
		if (types[i] == e_dynamicMass)
		{
			m_f[i] += m_gravity;
		}
	}

	// Compute non-zero Jacobians Jx, Jv
	b3Mat33* Jx = (b3Mat33*)m_allocator->Allocate(spring_size * sizeof(b3Mat33));
	b3SetZero_Jacobian(Jx, spring_size);

	b3Mat33* Jv = (b3Mat33*)m_allocator->Allocate(spring_size * sizeof(b3Mat33));
	b3SetZero_Jacobian(Jv, spring_size);

	// Compute forces and Jacobians
	for (u32 i = 0; i < m_springCount; ++i)
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
		Jx[i] = Jx11;

		// Compute Jv11
		b3Mat33 Jv11 = -S->kd * I;
		Jv[i] = Jv11;
	}
	
	// Solve Ax = b

	// Compute b

	// b = h * (f0 + h * dfdx * v0 + dfdx * y) ) 
	b3Vec3* b = (b3Vec3*) m_allocator->Allocate(size * sizeof(b3Vec3));

	// Jx_v = dfdx * v
	b3Vec3* Jx_v = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	// b3Mul(Jx_v, dfdx, v, size);
	b3Mul_Jacobian(Jx_v, m_v, size, Jx, m_springs, m_springCount);

	// Jx_v0y = dfdx * y
	b3Vec3* Jx_y = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	// b3Mul(Jx_y, dfdx, y, size);
	b3Mul_Jacobian(Jx_y, m_y, size, Jx, m_springs, m_springCount);

	// b = h * (f0 + h * Jx_v + Jx_y )
	for (u32 i = 0; i < size; ++i)
	{
		b[i] = h * (m_f[i] + h * Jx_v[i] + Jx_y[i]);
	}

	m_allocator->Free(Jx_y);
	m_allocator->Free(Jx_v);

	// Solve Ax = b
	b3Vec3* z = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* dv = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* Adv = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* r = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* c = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* q = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	
	b3Vec3* s = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* P = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	b3Vec3* inv_P = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));

	// Compute z
	b3Make_z(z, size, types, m_collisions);

	// dv = z
	b3Copy(dv, z, size);

	// Adv = A * dv 
	// b3Mul(Adv, A, dv, size);
	b3Mul_A(Adv, dv, size, m_allocator, m_inv_m, h, Jx, Jv, m_springs, m_springCount);

	// Compute P, P^-1
	// We compute A because P = diag(A)^-1
	// Note this is not necessary, and should be optimized as soon 
	// as possible.

	// A = M - h * dfdv - h * h * dfdx
	b3Mat33* A = (b3Mat33*)m_allocator->Allocate(size * size * sizeof(b3Mat33));

	// A = 0
	b3SetZero(A, size);

	// Compute dfdx, dfdv
	b3Mat33* dfdx = (b3Mat33*)m_allocator->Allocate(size * size * sizeof(b3Mat33));
	b3SetZero(dfdx, size);

	b3Mat33* dfdv = (b3Mat33*)m_allocator->Allocate(size * size * sizeof(b3Mat33));
	b3SetZero(dfdv, size);

	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* S = m_springs + i;

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
		float32 m = 1.0f / m_inv_m[i];

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

	for (u32 i = 0; i < size; ++i)
	{
		b3Mat33 D = A[B3_INDEX(i, i, size)];

		B3_ASSERT(D[0][0] != 0.0f);
		B3_ASSERT(D[1][1] != 0.0f);
		B3_ASSERT(D[2][2] != 0.0f);

		P[i] = b3Vec3(1.0f / D[0][0], 1.0f / D[1][1], 1.0f / D[2][2]);
		inv_P[i] = b3Vec3(D[0][0], D[1][1], D[2][2]);
	}

	m_allocator->Free(dfdv);
	m_allocator->Free(dfdx);
	m_allocator->Free(A);

	// eps0 = dot( filter(b), P * filter(b) )
	b3Vec3* filter_b = (b3Vec3*)m_allocator->Allocate(size * sizeof(b3Vec3));
	b3Filter(filter_b, b, size, types, m_collisions);

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

	// r = filter(b - Adv)
	b3Sub(r, b, Adv, size);
	b3Filter(r, r, size, types, m_collisions);

	// c = filter(P^-1 * r)
	for (u32 i = 0; i < m_massCount; ++i)
	{
		c[i][0] = inv_P[i][0] * r[i][0];
		c[i][1] = inv_P[i][1] * r[i][1];
		c[i][2] = inv_P[i][2] * r[i][2];
	}
	b3Filter(c, c, size, types, m_collisions);

	// epsNew = dot(r, c)
	float32 epsNew = b3Dot(r, c, size);

	// This is in [0, 1]
	// Making it smaller can increase accuracy, but it might increase the number 
	// of iterations to be taken by the solver.
	const float32 kTol = 0.25f;

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 200;
	
	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters && epsNew > kTol * kTol * eps0)
	{
		// q = filter(A * c) 
		// b3Mul(q, A, c, size);
		b3Mul_A(q, c, size, m_allocator, m_inv_m, h, Jx, Jv, m_springs, m_springCount);
		b3Filter(q, q, size, types, m_collisions);

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
		b3Filter(c, c, size, types, m_collisions);

		++iter;
	}

	m_step.iterations = iter;

	// Update state
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_v[i] += dv[i];
		m_x[i] += h * m_v[i] + m_y[i];
	}

	// Clear forces
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_f[i].SetZero();
	}

	// Clear position alteration
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_y[i].SetZero();
	}
	
	m_allocator->Free(inv_P);
	m_allocator->Free(P);
	m_allocator->Free(s);
	m_allocator->Free(q);
	m_allocator->Free(c);
	m_allocator->Free(r);
	m_allocator->Free(Adv);
	m_allocator->Free(dv);
	m_allocator->Free(z);
	m_allocator->Free(b);
	m_allocator->Free(Jv);
	m_allocator->Free(Jx);
}

void b3SpringCloth::Apply() const
{
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_mesh->vertices[i] = m_x[i];
	}
}

void b3SpringCloth::Draw(b3Draw* draw) const
{
	for (u32 i = 0; i < m_sphereCount; ++i)
	{
		draw->DrawSolidSphere(m_spheres[i].vertex, m_spheres[i].radius, b3Color_white);
	}

	const b3Mesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		draw->DrawPoint(m_x[i], 2.0f, b3Color_green);
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Vec3 v1 = m_x[t->v1];
		b3Vec3 v2 = m_x[t->v2];
		b3Vec3 v3 = m_x[t->v3];

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();

		b3Vec3 n2 = -n1;

		draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);
		draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}