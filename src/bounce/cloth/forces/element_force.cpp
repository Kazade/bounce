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

#include <bounce/cloth/forces/element_force.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

// Compute the orthotropic elastic tensor given Young Modulus and Poisson's Ratio.
// This is a 3x3 matrix.
static B3_FORCE_INLINE b3Mat33 b3ComputeC(scalar Ex, scalar Ey, scalar Es,
	scalar nu_xy, scalar nu_yx)
{
	scalar s = scalar(1) - nu_xy * nu_yx;
	B3_ASSERT(s != scalar(0));

	b3Mat33 C;
	C.x.x = Ex / s;
	C.x.y = Ey * nu_xy / s;
	C.x.z = scalar(0);

	C.y.x = Ex * nu_yx / s;
	C.y.y = Ey / s;
	C.y.z = scalar(0);

	C.z.x = scalar(0);
	C.z.y = scalar(0);
	C.z.z = Es;

	return C;
}

// Compute B = S * N,
// where S is the operational matrix and N are the shape functions.
// This is a 3 x 6 matrix.
// B = [dN1/dx	0		dN2/dx	0		dN3/dx	0]
//     [0       dN1/dy  0       dN2/dy  0       dN3/dy] 
//     [dN1/dy  dN1/dx  dN2/dy  dN2/dx  dN3/dy  dN3/dx]
//
// The shape functions are the Barycentric coordinates for point to a triangle in 2D:
// 
// N_1 = (a1 + b1 * x + c1 * y) / 2*A
// N_2 = (a2 + b2 * x + c2 * y) / 2*A
// N_3 = (a3 + b3 * x + c3 * y) / 2*A
//
// a1 = x2 * y3 - x3 * y2
// a2 = x3 * y1 - x1 * y3
// a3 = x1 * y2 - x2 * y1
// 
// b1 = y2 - y3
// b2 = y3 - y1
// b3 = y1 - y2
// 
// c1 = x3 - x2
// c2 = x1 - x3
// c3 = x2 - x1
static B3_FORCE_INLINE void b3ComputeB(scalar out[18],
	const b3Vec2& p1, const b3Vec2& p2, const b3Vec2& p3, 
	scalar invA2)
{
	scalar x1 = p1.x, y1 = p1.y;
	scalar x2 = p2.x, y2 = p2.y;
	scalar x3 = p3.x, y3 = p3.y;

	scalar dN1_dx = invA2 * (y2 - y3);
	scalar dN1_dy = invA2 * (x3 - x2);

	scalar dN2_dx = invA2 * (y3 - y1);
	scalar dN2_dy = invA2 * (x1 - x3);

	scalar dN3_dx = invA2 * (y1 - y2);
	scalar dN3_dy = invA2 * (x2 - x1);

	scalar B[18]
	{
		dN1_dx, 0, dN1_dy,
		0, dN1_dy, dN1_dx,

		dN2_dx, 0, dN2_dy,
		0, dN2_dy, dN2_dx,

		dN3_dx, 0, dN3_dy,
		0, dN3_dy, dN3_dx,
	};

	for (u32 i = 0; i < 18; ++i)
	{
		out[i] = B[i];
	}
}

// Extract rotation from a matrix.
static B3_FORCE_INLINE b3Mat22 b3ExtractRotation(const b3Mat22& M)
{
	// Polar Decomposition
	// https://research.cs.wisc.edu/graphics/Courses/838-s2002/Papers/polar-decomp.pdf
	scalar m11 = M.x.x, m12 = M.y.x;
	scalar m21 = M.x.y, m22 = M.y.y;

	scalar det = m11 * m22 - m12 * m21;

	b3Mat22 A;
	A.x.x = M.y.y;
	A.x.y = -M.y.x;
	A.y.x = -M.x.y;
	A.y.y = M.x.x;

	b3Mat22 Q = M + b3Sign(det) * A;

	Q.x.Normalize();
	Q.y.Normalize();

	return Q;
}

static B3_FORCE_INLINE scalar& b3GetElement(b3Mat22 K[9], u32 i, u32 j)
{
	B3_ASSERT(i < 6);
	B3_ASSERT(j < 6);

	u32 i0 = i / 2;
	u32 j0 = j / 2;

	b3Mat22& a = K[i0 + 3 * j0];

	u32 ii = i - 2 * i0;
	u32 jj = j - 2 * j0;

	return a(ii, jj);
}

// Convert a 6x6 matrix to 2x2 block form.
static B3_FORCE_INLINE void b3SetK(b3Mat22 K[9], scalar Ke[36])
{
	for (u32 i = 0; i < 6; ++i)
	{
		for (u32 j = 0; j < 6; ++j)
		{
			scalar k1 = Ke[i + 6 * j];
			scalar& k2 = b3GetElement(K, i, j);

			k2 = k1;
		}
	}
}

b3ElementForce::b3ElementForce(const b3ElementForceDef* def)
{
	m_type = e_elementForce;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_E_x = def->E_x;
	m_E_y = def->E_y;
	m_E_s = def->E_s;
	m_nu_xy = def->nu_xy;
	m_nu_yx = def->nu_yx;

	b3Vec3 p1 = def->v1;
	b3Vec3 p2 = def->v2;
	b3Vec3 p3 = def->v3;

	b3Vec3 n = b3Cross(p2 - p1, p3 - p1);
	scalar A2 = n.Normalize();
	
	// Create a basis B = [px py n] for the plane.  
	// B rotates vectors from plane space to world space.
	b3Vec3 px, py;
	b3ComputeBasis(n, px, py);

	// 2x3	
	scalar P[6] =
	{
		px.x, py.x,
		px.y, py.y,
		px.z, py.z
	};

	b3Vec2 x1, x2, x3;
	b3Mul(&x1.x, P, 2, 3, &p1.x, 3, 1);
	b3Mul(&x2.x, P, 2, 3, &p2.x, 3, 1);
	b3Mul(&x3.x, P, 2, 3, &p3.x, 3, 1);

	// Store initial positions in 2D
	m_x1 = x1;
	m_x2 = x2;
	m_x3 = x3;

	b3Mat22 S;
	S.x = x2 - x1;
	S.y = x3 - x1;

	// S^-1
	B3_ASSERT(A2 > scalar(0));
	scalar invA2 = scalar(1) / A2;
	m_invS = b3Inverse(S);

	// Area
	m_A = scalar(0.5) * A2;

	// 3x3
	m_C = b3ComputeC(m_E_x, m_E_y, m_E_s, m_nu_xy, m_nu_yx);

	// 3x6
	b3ComputeB(m_B, x1, x2, x3, invA2);

	// 6x3
	scalar BT[18];
	b3Transpose(BT, m_B, 3, 6);

	// 6x3
	scalar BT_C[18];
	b3Mul(BT_C, BT, 6, 3, &m_C.x.x, 3, 3);

	// 6x6
	scalar K[36];
	b3Mul(K, BT_C, 6, 3, m_B, 3, 6);
	for (u32 i = 0; i < 36; ++i)
	{
		K[i] *= m_A;
	}

	// Convert to block form
	b3SetK(m_K, K);
}

b3ElementForce::~b3ElementForce()
{

}

bool b3ElementForce::HasParticle(const b3ClothParticle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle;
}

// https://animation.rwth-aachen.de/media/papers/2013-CAG-AdaptiveCloth.pdf
void b3ElementForce::Apply(const b3ClothForceSolverData* data)
{
	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;
	u32 i3 = m_p3->m_solverId;

	b3DenseVec3& p = *data->x;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;

	b3Vec3 p1 = p[i1];
	b3Vec3 p2 = p[i2];
	b3Vec3 p3 = p[i3];

	b3Vec3 n = b3Cross(p2 - p1, p3 - p1);
	n.Normalize();

	b3Vec3 px, py;
	b3ComputeBasis(n, px, py);

	// 2x3
	scalar P[6] =
	{
		px.x, py.x,
		px.y, py.y,
		px.z, py.z
	};

	// 3x2
	scalar PT[6] =
	{
		px.x, px.y, px.z,
		py.x, py.y, py.z
	};

	// Project the positions to 2D
	b3Vec2 x1, x2, x3;
	b3Mul(&x1.x, P, 2, 3, &p1.x, 3, 1);
	b3Mul(&x2.x, P, 2, 3, &p2.x, 3, 1);
	b3Mul(&x3.x, P, 2, 3, &p3.x, 3, 1);

	b3Mat22 T;
	T.x = x2 - x1;
	T.y = x3 - x1;

	// Deformation gradient
	b3Mat22 A = T * m_invS;

	// Extract rotation
	b3Mat22 R = b3ExtractRotation(A);

	// Inverse rotation
	b3Mat22 RT = b3Transpose(R);

	// 2D displacements in unrotated frame
	b3Vec2 us[3];
	us[0] = RT * x1 - m_x1;
	us[1] = RT * x2 - m_x2;
	us[2] = RT * x3 - m_x3;

	// 2D forces in unrotated frame
	b3Vec2 fs[3];

	for (u32 i = 0; i < 3; ++i)
	{
		fs[i].SetZero();
		
		for (u32 j = 0; j < 3; ++j)
		{
			b3Mat22 k = m_K[i + 3 * j];

			fs[i] += k * us[j];
		}
	}

	// Rotate the forces to deformed frame
	fs[0] = R * fs[0];
	fs[1] = R * fs[1];
	fs[2] = R * fs[2];

	// Project the forces to 3D
	b3Vec3 f1, f2, f3;
	b3Mul(&f1.x, PT, 3, 2, &fs[0].x, 2, 1);
	b3Mul(&f2.x, PT, 3, 2, &fs[1].x, 2, 1);
	b3Mul(&f3.x, PT, 3, 2, &fs[2].x, 2, 1);

	// Negate f
	f[i1] -= f1;
	f[i2] -= f2;
	f[i3] -= f3;

	// 3D corotated stiffness matrix
	u32 vs[3] = { i1, i2, i3 };

	for (u32 i = 0; i < 3; ++i)
	{
		u32 vi = vs[i];

		for (u32 j = 0; j < 3; ++j)
		{
			u32 vj = vs[j];

			// 2D corotated stiffness matrix
			b3Mat22 k = R * m_K[i + 3 * j] * RT;

			// In the paper, Jan uses P^T * R.
			// Here, I use 
			// K = P^T * k * P
			// We can do both ways, but 
			// the latter is more practical than the former.

			// P^T * k = 3x2 * 2x2 = 3x2
			scalar PT_k[6];
			b3Mul(PT_k, PT, 3, 2, &k.x.x, 2, 2);

			// (P^T * k) * P = 3x2 * 2x3 = 3x3
			b3Mat33 Ke;
			b3Mul(&Ke.x.x, PT_k, 3, 2, P, 2, 3);

			// Negate K 
			dfdx(vi, vj) -= Ke;
		}
	}
}