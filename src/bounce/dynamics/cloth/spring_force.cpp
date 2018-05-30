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

#include <bounce/dynamics/cloth/spring_force.h>
#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/cloth_solver.h>

void b3SpringForceDef::Initialize(b3Particle* particle1, b3Particle* particle2, float32 structuralStiffness, float32 dampingStiffness)
{
	type = e_springForce;
	p1 = particle1;
	p2 = particle2;
	b3Vec3 x1 = p1->GetPosition();
	b3Vec3 x2 = p2->GetPosition();
	restLength = b3Distance(x1, x2);
	structural = structuralStiffness;
	damping = dampingStiffness;
}

b3SpringForce::b3SpringForce(const b3SpringForceDef* def)
{
	m_type = e_springForce;
	m_p1 = def->p1;
	m_p2 = def->p2;
	m_L0 = def->restLength;
	m_ks = def->structural;
	m_kd = def->damping;
	m_f.SetZero();
}

b3SpringForce::~b3SpringForce()
{

}

void b3SpringForce::Initialize(const b3ClothSolverData* data)
{
	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;

	b3Vec3 x1 = data->x[i1];
	b3Vec3 v1 = data->v[i1];

	b3Vec3 x2 = data->x[i2];
	b3Vec3 v2 = data->v[i2];

	b3Mat33 I; I.SetIdentity();

	b3Vec3 dx = x1 - x2;

	float32 L = b3Length(dx);

	if (L >= m_L0)
	{
		b3Vec3 n = dx / L;

		// Tension
		m_f = -m_ks * (L - m_L0) * n;

		// Jacobian
		m_Jx = -m_ks * (b3Outer(dx, dx) + (1.0f - m_L0 / L) * (I - b3Outer(dx, dx)));
	}
	else
	{
		m_f.SetZero();
		m_Jx.SetZero();
	}

	// Damping
	b3Vec3 dv = v1 - v2;

	m_f += -m_kd * dv;
	m_Jv = -m_kd * I;
}

void b3SpringForce::Apply(const b3ClothSolverData* data)
{
	b3Vec3* f = data->f;
	b3SymMat33& dfdx = *data->dfdx;
	b3SymMat33& dfdv = *data->dfdv;

	u32 i1 = m_p1->m_solverId;
	u32 i2 = m_p2->m_solverId;

	f[i1] += m_f;
	f[i2] -= m_f;

	b3Mat33 Jx11 = m_Jx;
	b3Mat33 Jx12 = -Jx11;
	b3Mat33 Jx21 = Jx12;
	b3Mat33 Jx22 = Jx11;

	dfdx(i1, i1) += Jx11;
	dfdx(i1, i2) += Jx12;
	dfdx(i2, i1) += Jx21;
	dfdx(i2, i2) += Jx22;

	b3Mat33 Jv11 = m_Jv;
	b3Mat33 Jv12 = -Jv11;
	b3Mat33 Jv21 = Jv12;
	b3Mat33 Jv22 = Jv11;

	dfdv(i1, i1) += Jv11;
	dfdv(i1, i2) += Jv12;
	dfdv(i2, i1) += Jv21;
	dfdv(i2, i2) += Jv22;
}