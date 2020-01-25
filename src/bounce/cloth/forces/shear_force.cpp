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

#include <bounce/cloth/forces/shear_force.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

void b3ShearForceDef::Initialize(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3)
{
	b3Vec3 A = p1, B = p2, C = p3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	// (u, v) 1
	u1 = scalar(0);
	v1 = scalar(0);

	// (u, v) 2
	u2 = b3Length(AB);
	v2 = scalar(0);

	// (u, v) 3
	B3_ASSERT(u2 > scalar(0));
	b3Vec3 n_AB = AB / u2;

	// a  = b * h / 2
	// h = (a * 2) / b
	scalar a2 = b3Length(b3Cross(AB, AC));
	B3_ASSERT(a2 > scalar(0));

	u3 = b3Dot(AC, n_AB);
	v3 = a2 / u2;
	
	alpha = scalar(0.5) * a2;
}

b3ShearForce::b3ShearForce(const b3ShearForceDef* def)
{
	m_type = e_shearForce;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_ks = def->shearing;
	m_kd = def->damping;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
	m_alpha = def->alpha;

	scalar u1 = def->u1, v1 = def->v1;
	scalar u2 = def->u2, v2 = def->v2;
	scalar u3 = def->u3, v3 = def->v3;

	// (u, v) matrix
	scalar du1 = u2 - u1;
	scalar dv1 = v2 - v1;
	scalar du2 = u3 - u1;
	scalar dv2 = v3 - v1;

	m_du1 = du1;
	m_dv1 = dv1;
	m_du2 = du2;
	m_dv2 = dv2;

	scalar det = du1 * dv2 - du2 * dv1;
	B3_ASSERT(det != scalar(0));
	m_inv_det = scalar(1) / det;

	scalar inv_det = m_inv_det;

	m_dwudx.x = inv_det * (dv1 - dv2);
	m_dwudx.y = inv_det * dv2;
	m_dwudx.z = -inv_det * dv1;

	m_dwvdx.x = inv_det * (du2 - du1);
	m_dwvdx.y = -inv_det * du2;
	m_dwvdx.z = inv_det * du1;
}

b3ShearForce::~b3ShearForce()
{

}

bool b3ShearForce::HasParticle(const b3ClothParticle* particle) const
{
	return m_p1 == particle || m_p2 == particle || m_p3 == particle;
}

void b3ShearForce::Apply(const b3ClothForceSolverData* data)
{
	scalar alpha = m_alpha;
	scalar du1 = m_du1;
	scalar dv1 = m_dv1;
	scalar du2 = m_du2;
	scalar dv2 = m_dv2;
	scalar inv_det = m_inv_det;
	b3Vec3 dwudx = m_dwudx;
	b3Vec3 dwvdx = m_dwvdx;

	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;
	u32 i3 = m_p3->m_solverId;

	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3Vec3 x1 = x[i1];
	b3Vec3 x2 = x[i2];
	b3Vec3 x3 = x[i3];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];

	b3Mat33 I; I.SetIdentity();

	b3Vec3 dx1 = x2 - x1;
	b3Vec3 dx2 = x3 - x1;

	b3Vec3 wu = inv_det * (dv2 * dx1 - dv1 * dx2);
	b3Vec3 wv = inv_det * (-du2 * dx1 + du1 * dx2);

	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();

	// Jacobian
	b3Vec3 dCdx[3];
	for (u32 i = 0; i < 3; ++i)
	{
		dCdx[i] = alpha * (dwudx[i] * wv + dwvdx[i] * wu);
	}

	if (m_ks > scalar(0))
	{
		scalar C = alpha * b3Dot(wu, wv);

		// Force
		b3Vec3 fs[3];
		for (u32 i = 0; i < 3; ++i)
		{
			fs[i] = -m_ks * C * dCdx[i];
		}

		m_f1 += fs[0];
		m_f2 += fs[1];
		m_f3 += fs[2];

		// Force derivative
		b3Mat33 K[3][3];
		for (u32 i = 0; i < 3; ++i)
		{
			for (u32 j = 0; j < 3; ++j)
			{
				b3Mat33 d2Cxij = alpha * (dwudx[i] * dwvdx[j] + dwudx[j] * dwvdx[i]) * I;
				b3Mat33 Kij = -m_ks * (b3Outer(dCdx[i], dCdx[j]) + C * d2Cxij);
				
				K[i][j] = Kij;
			}
		}

		dfdx(i1, i1) += K[0][0];
		dfdx(i1, i2) += K[0][1];
		dfdx(i1, i3) += K[0][2];

		dfdx(i2, i1) += K[1][0];
		dfdx(i2, i2) += K[1][1];
		dfdx(i2, i3) += K[1][2];

		dfdx(i3, i1) += K[2][0];
		dfdx(i3, i2) += K[2][1];
		dfdx(i3, i3) += K[2][2];
	}

	if (m_kd > scalar(0))
	{
		b3Vec3 vs[3] = { v1, v2, v3 };

		scalar dCdt = scalar(0);
		for (u32 i = 0; i < 3; ++i) 
		{
			dCdt += b3Dot(dCdx[i], vs[i]);
		}

		// Force
		b3Vec3 fs[3];
		for (u32 i = 0; i < 3; ++i)
		{
			fs[i] = -m_kd * dCdt * dCdx[i];
		}

		m_f1 += fs[0];
		m_f2 += fs[1];
		m_f3 += fs[2];

		// Force derivative
		b3Mat33 K[3][3];
		for (u32 i = 0; i < 3; ++i)
		{
			for (u32 j = 0; j < 3; ++j)
			{
				b3Mat33 Kij = -m_kd * b3Outer(dCdx[i], dCdx[j]);

				K[i][j] = Kij;
			}
		}

		dfdv(i1, i1) += K[0][0];
		dfdv(i1, i2) += K[0][1];
		dfdv(i1, i3) += K[0][2];

		dfdv(i2, i1) += K[1][0];
		dfdv(i2, i2) += K[1][1];
		dfdv(i2, i3) += K[1][2];

		dfdv(i3, i1) += K[2][0];
		dfdv(i3, i2) += K[2][1];
		dfdv(i3, i3) += K[2][2];
	}
	
	f[i1] += m_f1;
	f[i2] += m_f2;
	f[i3] += m_f3;
}