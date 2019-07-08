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

#include <bounce/cloth/forces/mouse_force.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/cloth_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

b3MouseForce::b3MouseForce(const b3MouseForceDef* def)
{
	m_type = e_mouseForce;
	m_particle = def->particle;
	m_triangle = def->triangle;
	m_w2 = def->w2;
	m_w3 = def->w3;
	m_w4 = def->w4;
	m_km = def->mouse;
	m_kd = def->damping;
	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
	m_f4.SetZero();
}

b3MouseForce::~b3MouseForce()
{

}

bool b3MouseForce::HasParticle(const b3Particle* particle) const
{
	b3Cloth* cloth = m_triangle->m_cloth;
	u32 triangleIndex = m_triangle->m_triangle;
	b3ClothMeshTriangle* triangle = cloth->m_mesh->triangles + triangleIndex;

	b3Particle* p1 = cloth->m_particles[triangle->v1];
	b3Particle* p2 = cloth->m_particles[triangle->v2];
	b3Particle* p3 = cloth->m_particles[triangle->v3];

	return m_particle == particle || p1 == particle || p2 == particle || p3 == particle;
}

void b3MouseForce::Apply(const b3ClothForceSolverData* data)
{
	b3Cloth* cloth = m_triangle->m_cloth;
	u32 triangleIndex = m_triangle->m_triangle;
	b3ClothMeshTriangle* triangle = cloth->m_mesh->triangles + triangleIndex;

	b3Particle* p1 = m_particle;
	b3Particle* p2 = cloth->m_particles[triangle->v1];
	b3Particle* p3 = cloth->m_particles[triangle->v2];
	b3Particle* p4 = cloth->m_particles[triangle->v3];

	u32 i1 = p1->m_solverId;
	u32 i2 = p2->m_solverId;
	u32 i3 = p3->m_solverId;
	u32 i4 = p4->m_solverId;

	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3Vec3 x1 = x[i1];
	b3Vec3 x2 = x[i2];
	b3Vec3 x3 = x[i3];
	b3Vec3 x4 = x[i4];

	b3Vec3 v1 = v[i1];
	b3Vec3 v2 = v[i2];
	b3Vec3 v3 = v[i3];
	b3Vec3 v4 = v[i4];

	b3Mat33 I; I.SetIdentity();

	m_f1.SetZero();
	m_f2.SetZero();
	m_f3.SetZero();
	m_f4.SetZero();

	b3Vec3 c2 = m_w2 * x2 + m_w3 * x3 + m_w4 * x4;

	b3Vec3 d = x1 - c2;
	float32 len = b3Length(d);

	if (len > 0.0f)
	{
		b3Vec3 n = d / len;

		// Jacobian
		b3Vec3 dCdx[4];
		dCdx[0] = n;
		dCdx[1] = -m_w2 * n;
		dCdx[2] = -m_w3 * n;
		dCdx[3] = -m_w4 * n;

		if (m_km > 0.0f)
		{
			float32 C = len;

			// Force
			b3Vec3 fs[4];
			for (u32 i = 0; i < 4; ++i)
			{
				fs[i] = -m_km * C * dCdx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];
			m_f4 += fs[3];

			// Force derivative
			b3Mat33 K[4][4];
			for (u32 i = 0; i < 4; ++i)
			{
				for (u32 j = 0; j < 4; ++j)
				{
					//b3Mat33 d2Cvxij;
					//b3Mat33 Kij = -m_km * (b3Outer(dCdx[i], dCdx[j]) + C * d2Cvxij);
					b3Mat33 Kij = -m_km * b3Outer(dCdx[i], dCdx[j]);

					K[i][j] = Kij;
				}
			}

			dfdx(i1, i1) += K[0][0];
			dfdx(i1, i2) += K[0][1];
			dfdx(i1, i3) += K[0][2];
			dfdx(i1, i4) += K[0][3];

			dfdx(i2, i1) += K[1][0];
			dfdx(i2, i2) += K[1][1];
			dfdx(i2, i3) += K[1][2];
			dfdx(i2, i4) += K[1][3];

			dfdx(i3, i1) += K[2][0];
			dfdx(i3, i2) += K[2][1];
			dfdx(i3, i3) += K[2][2];
			dfdx(i3, i4) += K[2][3];

			dfdx(i4, i1) += K[3][0];
			dfdx(i4, i2) += K[3][1];
			dfdx(i4, i3) += K[3][2];
			dfdx(i4, i4) += K[3][3];
		}
		
		if (m_kd > 0.0f)
		{
			b3Vec3 vs[4] = { v1, v2, v3, v4 };

			float32 dCdt = 0.0f;
			for (u32 i = 0; i < 4; ++i)
			{
				dCdt += b3Dot(dCdx[i], vs[i]);
			}

			// Force
			b3Vec3 fs[4];
			for (u32 i = 0; i < 4; ++i)
			{
				fs[i] = -m_kd * dCdt * dCdx[i];
			}

			m_f1 += fs[0];
			m_f2 += fs[1];
			m_f3 += fs[2];
			m_f4 += fs[3];

			// Force derivative
			b3Mat33 K[4][4];
			for (u32 i = 0; i < 4; ++i)
			{
				for (u32 j = 0; j < 4; ++j)
				{
					b3Mat33 Kij = -m_kd * b3Outer(dCdx[i], dCdx[j]);

					K[i][j] = Kij;
				}
			}

			dfdv(i1, i1) += K[0][0];
			dfdv(i1, i2) += K[0][1];
			dfdv(i1, i3) += K[0][2];
			dfdv(i1, i4) += K[0][3];

			dfdv(i2, i1) += K[1][0];
			dfdv(i2, i2) += K[1][1];
			dfdv(i2, i3) += K[1][2];
			dfdv(i2, i4) += K[1][3];

			dfdv(i3, i1) += K[2][0];
			dfdv(i3, i2) += K[2][1];
			dfdv(i3, i3) += K[2][2];
			dfdv(i3, i4) += K[2][3];

			dfdv(i4, i1) += K[3][0];
			dfdv(i4, i2) += K[3][1];
			dfdv(i4, i3) += K[3][2];
			dfdv(i4, i4) += K[3][3];
		}
	}

	f[i1] += m_f1;
	f[i2] += m_f2;
	f[i3] += m_f3;
	f[i4] += m_f4;
}