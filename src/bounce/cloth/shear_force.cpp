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

#include <bounce/cloth/shear_force.h>
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

b3ShearForce::b3ShearForce(const b3ShearForceDef* def)
{
	m_type = e_shearForce;
	m_triangle = def->triangle;
	m_ks = def->shearing;
	m_kd = def->damping;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
}

b3ShearForce::~b3ShearForce()
{

}

bool b3ShearForce::HasParticle(const b3Particle* particle) const
{
	b3Cloth* cloth = m_triangle->m_cloth;
	u32 triangleIndex = m_triangle->m_triangle;
	b3ClothMeshTriangle* triangle = cloth->m_mesh->triangles + triangleIndex;

	b3Particle* p1 = cloth->m_particles[triangle->v1];
	b3Particle* p2 = cloth->m_particles[triangle->v2];
	b3Particle* p3 = cloth->m_particles[triangle->v3];

	return p1 == particle || p2 == particle || p3 == particle;
}

void b3ShearForce::Apply(const b3ClothForceSolverData* data)
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

	// Jacobian
	b3Vec3 dCdx[3];
	for (u32 i = 0; i < 3; ++i)
	{
		dCdx[i] = alpha * (dwudx[i] * wv + dwvdx[i] * wu);
	}

	if (m_ks > 0.0f)
	{
		float32 C = alpha * b3Dot(wu, wv);

		// Force
		b3Vec3 fs[3];
		for (u32 i = 0; i < 3; ++i)
		{
			fs[i] = -m_ks * C * dCdx[i];
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
				//b3Mat33 d2Cxij = alpha * (dwudx[i] * dwvdx[j] + dwudx[j] * dwvdx[i]) * I;
				//b3Mat33 Jij = -m_ks * (b3Outer(dCdx[i], dCdx[j]) + C * d2Cxij);
				b3Mat33 Jij = -m_ks * b3Outer(dCdx[i], dCdx[j]);

				J[i][j] = Jij;
			}
		}

		dfdx(i1, i1) += J[0][0];
		dfdx(i1, i2) += J[0][1];
		dfdx(i1, i3) += J[0][2];

		dfdx(i2, i1) += J[1][0];
		dfdx(i2, i2) += J[1][1];
		dfdx(i2, i3) += J[1][2];

		dfdx(i3, i1) += J[2][0];
		dfdx(i3, i2) += J[2][1];
		dfdx(i3, i3) += J[2][2];
	}

	if (m_kd > 0.0f)
	{
		b3Vec3 vs[3] = { v1, v2, v3 };

		float32 dCdt = 0.0f;
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

		// Jacobian
		b3Mat33 J[3][3];
		for (u32 i = 0; i < 3; ++i)
		{
			for (u32 j = 0; j < 3; ++j)
			{
				b3Mat33 Jij = -m_kd * b3Outer(dCdx[i], dCdx[j]);

				J[i][j] = Jij;
			}
		}

		dfdv(i1, i1) += J[0][0];
		dfdv(i1, i2) += J[0][1];
		dfdv(i1, i3) += J[0][2];

		dfdv(i2, i1) += J[1][0];
		dfdv(i2, i2) += J[1][1];
		dfdv(i2, i3) += J[1][2];

		dfdv(i3, i1) += J[2][0];
		dfdv(i3, i2) += J[2][1];
		dfdv(i3, i3) += J[2][2];
	}
	
	f[i1] += m_f1;
	f[i2] += m_f2;
	f[i3] += m_f3;
}