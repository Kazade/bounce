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

#include <bounce/dynamics/cloth/bend_force.h>
#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/sparse_sym_mat33.h>

void b3BendForceDef::Initialize(b3Particle* particle1, b3Particle* particle2, b3Particle* particle3, b3Particle* particle4, float32 structuralStiffness, float32 dampingStiffness)
{
	type = e_bendForce;
	p1 = particle1;
	p2 = particle2;
	p3 = particle3;
	p4 = particle4;
	
	b3Vec3 x1 = p1->GetPosition();
	b3Vec3 x2 = p2->GetPosition();
	b3Vec3 x3 = p3->GetPosition();
	b3Vec3 x4 = p4->GetPosition();

	restDistance = b3Distance(x2, x3);
	structural = structuralStiffness;
	damping = dampingStiffness;
}

b3BendForce::b3BendForce(const b3BendForceDef* def)
{
	m_type = e_bendForce;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_p3 = def->p3;
	m_p4 = def->p4;
	m_L0 = def->restDistance;
	m_angle0 = def->restAngle;
	m_ks = def->structural;
	m_kd = def->damping;
}

b3BendForce::~b3BendForce()
{

}

void b3BendForce::Apply(const b3ClothSolverData* data)
{
	b3DenseVec3& x = *data->x;
	b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseSymMat33& dfdx = *data->dfdx;
	b3SparseSymMat33& dfdv = *data->dfdv;

	u32 i1 = m_p2->m_solverId;
	u32 i2 = m_p3->m_solverId;

	b3Vec3 x1 = x[i1];
	b3Vec3 v1 = v[i1];

	b3Vec3 x2 = x[i2];
	b3Vec3 v2 = v[i2];

	b3Mat33 I; I.SetIdentity();

	b3Vec3 ef; ef.SetZero();

	if (m_ks > 0.0f)
	{
		b3Vec3 dx = x1 - x2;

		float32 L = b3Length(dx);

		if (L >= m_L0)
		{
			// Apply tension
			b3Vec3 n = dx / L;

			ef += -m_ks * (L - m_L0) * n;

			// Jacobian
			b3Mat33 Jx11 = -m_ks * (b3Outer(dx, dx) + (1.0f - m_L0 / L) * (I - b3Outer(dx, dx)));
			b3Mat33 Jx12 = -Jx11;
			//b3Mat33 Jx21 = Jx12;
			b3Mat33 Jx22 = Jx11;

			dfdx(i1, i1) += Jx11;
			dfdx(i1, i2) += Jx12;
			//dfdx(i2, i1) += Jx21;
			dfdx(i2, i2) += Jx22;
		}
	}

	if (m_kd > 0.0f)
	{
		// Apply damping
		b3Vec3 dv = v1 - v2;

		ef += -m_kd * dv;

		b3Mat33 Jv11 = -m_kd * I;
		b3Mat33 Jv12 = -Jv11;
		//b3Mat33 Jv21 = Jv12;
		b3Mat33 Jv22 = Jv11;

		dfdv(i1, i1) += Jv11;
		dfdv(i1, i2) += Jv12;
		//dfdv(i2, i1) += Jv21;
		dfdv(i2, i2) += Jv22;
	}

	f[i1] += ef;
	f[i2] -= ef;
}