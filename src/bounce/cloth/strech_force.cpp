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

#include <bounce/cloth/strech_force.h>
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_sym_mat33.h>

b3StrechForce::b3StrechForce(const b3StrechForceDef* def)
{
	m_type = e_strechForce;
	m_triangle = def->triangle;
	m_ks = def->streching;
	m_kd = def->damping;
	m_bu = def->bu;
	m_bv = def->bv;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
}

b3StrechForce::~b3StrechForce()
{

}

bool b3StrechForce::HasParticle(const b3Particle* particle) const
{
	b3Cloth* cloth = m_triangle->m_cloth;
	u32 triangleIndex = m_triangle->m_triangle;
	b3ClothMeshTriangle* triangle = cloth->m_mesh->triangles + triangleIndex;

	b3Particle* p1 = cloth->m_particles[triangle->v1];
	b3Particle* p2 = cloth->m_particles[triangle->v2];
	b3Particle* p3 = cloth->m_particles[triangle->v3];

	return p1 == particle || p2 == particle || p3 == particle;
}

void b3StrechForce::Apply(const b3ClothForceSolverData* data)
{
	b3Cloth* cloth = m_triangle->m_cloth;
	u32 triangleIndex = m_triangle->m_triangle;
	b3ClothMeshTriangle* triangle = cloth->m_mesh->triangles + triangleIndex;

	float32 alpha = m_triangle->m_alpha;
	float32 du1 = m_triangle->m_du1;
	float32 dv1 = m_triangle->m_dv1;
	float32 du2 = m_triangle->m_du2;
	float32 dv2 = m_triangle->m_dv2;
	float32 inv_det = m_triangle->m_inv_det;

	b3Particle* p1 = cloth->m_particles[triangle->v1];
	b3Particle* p2 = cloth->m_particles[triangle->v2];
	b3Particle* p3 = cloth->m_particles[triangle->v3];

	u32 i1 = p1->m_solverId;
	u32 i2 = p2->m_solverId;
	u32 i3 = p3->m_solverId;

	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseSymMat33& dfdx = *data->dfdx;
	b3SparseSymMat33& dfdv = *data->dfdv;

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
	float32 len_wu = b3Length(wu);

	b3Vec3 wv = inv_det * (-du2 * dx1 + du1 * dx2);
	float32 len_wv = b3Length(wv);

	b3Vec3 dwudx;
	dwudx[0] = inv_det * (dv1 - dv2);
	dwudx[1] = inv_det * dv2;
	dwudx[2] = -inv_det * dv1;

	b3Vec3 dwvdx;
	dwvdx[0] = inv_det * (du2 - du1);
	dwvdx[1] = -inv_det * du2;
	dwvdx[2] = inv_det * du1;

	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();

	if (len_wu > 0.0f)
	{
		float32 inv_len_wu = 1.0f / len_wu;
		b3Vec3 n_wu = inv_len_wu * wu;

		// Jacobian
		b3Vec3 dCudx[3];
		for (u32 i = 0; i < 3; ++i)
		{
			dCudx[i] = alpha * dwudx[i] * n_wu;
		}

		if (m_ks > 0.0f)
		{
			if (len_wu > m_bu)
			{
				float32 Cu = alpha * (len_wu - m_bu);

				// Force
				b3Vec3 fs[3];
				for (u32 i = 0; i < 3; ++i)
				{
					fs[i] = -m_ks * Cu * dCudx[i];
				}

				m_f1 += fs[0];
				m_f2 += fs[1];
				m_f3 += fs[2];

				// Jacobian
				b3Mat33 J[3][3];
				for (u32 i = 0; i < 3; ++i)
				{
					for (u32 j = 0; j < 3; ++j)
					{
						b3Mat33 d2Cuxij = (alpha * inv_len_wu * dwudx[i] * dwudx[j]) * (I - b3Outer(n_wu, n_wu));

						b3Mat33 Jij = -m_ks * (b3Outer(dCudx[i], dCudx[j]) + Cu * d2Cuxij);

						J[i][j] = Jij;
					}
				}

				dfdx(i1, i1) += J[0][0];
				dfdx(i1, i2) += J[0][1];
				dfdx(i1, i3) += J[0][2];

				//dfdx(i2, i1) += J[1][0];
				dfdx(i2, i2) += J[1][1];
				dfdx(i2, i3) += J[1][2];

				//dfdx(i3, i1) += J[2][0];
				//dfdx(i3, i2) += J[2][1];
				dfdx(i3, i3) += J[2][2];
			}
		}

		if (m_kd > 0.0f)
		{
			b3Vec3 vs[3] = { v1, v2, v3 };
			float32 dCudt = 0.0f;
			for (u32 i = 0; i < 3; ++i)
			{
				dCudt += b3Dot(dCudx[i], vs[i]);
			}

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_kd * dCudt * dCudx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Jacobian
			b3Mat33 J[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Jij = -m_kd * b3Outer(dCudx[i], dCudx[j]);

					J[i][j] = Jij;
				}
			}

			dfdv(i1, i1) += J[0][0];
			dfdv(i1, i2) += J[0][1];
			dfdv(i1, i3) += J[0][2];

			//dfdv(i2, i1) += J[1][0];
			dfdv(i2, i2) += J[1][1];
			dfdv(i2, i3) += J[1][2];

			//dfdv(i3, i1) += J[2][0];
			//dfdv(i3, i2) += J[2][1];
			dfdv(i3, i3) += J[2][2];
		}
	}

	if (len_wv > 0.0f)
	{
		float32 inv_len_wv = 1.0f / len_wv;
		b3Vec3 n_wv = inv_len_wv * wv;

		// Jacobian
		b3Vec3 dCvdx[3];
		for (u32 i = 0; i < 3; ++i)
		{
			dCvdx[i] = alpha * dwvdx[i] * n_wv;
		}

		if (m_ks > 0.0f)
		{
			if (len_wv > m_bv)
			{
				float32 Cv = alpha * (len_wv - m_bv);

				// Force
				b3Vec3 fs[3];
				for (u32 i = 0; i < 3; ++i)
				{
					fs[i] = -m_ks * Cv * dCvdx[i];
				}

				m_f1 += fs[0];
				m_f2 += fs[1];
				m_f3 += fs[2];

				// Jacobian
				b3Mat33 J[3][3];
				for (u32 i = 0; i < 3; ++i)
				{
					for (u32 j = 0; j < 3; ++j)
					{
						b3Mat33 d2Cvxij = (alpha * inv_len_wv * dwvdx[i] * dwvdx[j]) * (I - b3Outer(n_wv, n_wv));

						b3Mat33 Jij = -m_ks * (b3Outer(dCvdx[i], dCvdx[j]) + Cv * d2Cvxij);

						J[i][j] = Jij;
					}
				}

				dfdx(i1, i1) += J[0][0];
				dfdx(i1, i2) += J[0][1];
				dfdx(i1, i3) += J[0][2];

				//dfdx(i2, i1) += J[1][0];
				dfdx(i2, i2) += J[1][1];
				dfdx(i2, i3) += J[1][2];

				//dfdx(i3, i1) += J[2][0];
				//dfdx(i3, i2) += J[2][1];
				dfdx(i3, i3) += J[2][2];
			}
		}

		if (m_kd > 0.0f)
		{
			b3Vec3 vs[3] = { v1, v2, v3 };

			float32 dCvdt = 0.0f;
			for (u32 i = 0; i < 3; ++i)
			{
				dCvdt += b3Dot(dCvdx[i], vs[i]);
			}

			// Force
			b3Vec3 fs[3];
			for (u32 i = 0; i < 3; ++i)
			{
				fs[i] = -m_kd * dCvdt * dCvdx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];

			// Jacobian
			b3Mat33 J[3][3];
			for (u32 i = 0; i < 3; ++i)
			{
				for (u32 j = 0; j < 3; ++j)
				{
					b3Mat33 Jij = -m_kd * b3Outer(dCvdx[i], dCvdx[j]);

					J[i][j] = Jij;
				}
			}

			dfdv(i1, i1) += J[0][0];
			dfdv(i1, i2) += J[0][1];
			dfdv(i1, i3) += J[0][2];

			//dfdv(i2, i1) += J[1][0];
			dfdv(i2, i2) += J[1][1];
			dfdv(i2, i3) += J[1][2];

			//dfdv(i3, i1) += J[2][0];
			//dfdv(i3, i2) += J[2][1];
			dfdv(i3, i3) += J[2][2];
		}
	}

	f[i1] += m_f1;
	f[i2] += m_f2;
	f[i3] += m_f3;
}