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

#include <testbed/framework/cloth_dragger.h>
#include <bounce/common/draw.h>

b3ClothDragger::b3ClothDragger(b3Ray3* ray, b3Cloth* cloth)
{
	m_spring = false;
	m_ray = ray;
	m_cloth = cloth;
	m_triangle = nullptr;
}

b3ClothDragger::~b3ClothDragger()
{

}

bool b3ClothDragger::StartDragging()
{
	B3_ASSERT(IsDragging() == false);

	b3ClothRayCastSingleOutput rayOut;
	if (m_cloth->RayCastSingle(&rayOut, m_ray->A(), m_ray->B()) == false)
	{
		return false;
	}

	m_mesh = m_cloth->GetMesh();
	m_triangle = m_mesh->triangles + rayOut.triangle;
	m_x = rayOut.fraction;

	b3Particle* p1 = m_cloth->GetVertexParticle(m_triangle->v1);
	b3Particle* p2 = m_cloth->GetVertexParticle(m_triangle->v2);
	b3Particle* p3 = m_cloth->GetVertexParticle(m_triangle->v3);

	b3Vec3 v1 = p1->GetPosition();
	b3Vec3 v2 = p2->GetPosition();
	b3Vec3 v3 = p3->GetPosition();

	b3Vec3 B = GetPointB();

	float32 wABC[4];
	b3BarycentricCoordinates(wABC, v1, v2, v3, B);

	if (wABC[3] > B3_EPSILON)
	{
		m_u = wABC[0] / wABC[3];
		m_v = wABC[1] / wABC[3];
	}
	else
	{
		m_u = m_v = 0.0f;
	}

	if (m_spring)
	{
		b3ParticleDef pd;
		pd.type = e_staticParticle;
		pd.position = B;

		m_particle = m_cloth->CreateParticle(pd);

		{
			b3SpringForceDef sfd;
			sfd.p1 = m_particle;
			sfd.p2 = p1;
			sfd.restLength = 0.0f;
			sfd.structural = 10000.0f;
			m_s1 = (b3SpringForce*)m_cloth->CreateForce(sfd);
		}

		{
			b3SpringForceDef sfd;
			sfd.p1 = m_particle;
			sfd.p2 = p2;
			sfd.restLength = 0.0f;
			sfd.structural = 10000.0f;
			m_s2 = (b3SpringForce*)m_cloth->CreateForce(sfd);
		}

		{
			b3SpringForceDef sfd;
			sfd.p1 = m_particle;
			sfd.p2 = p3;
			sfd.restLength = 0.0f;
			sfd.structural = 10000.0f;
			m_s3 = (b3SpringForce*)m_cloth->CreateForce(sfd);
		}
	}
	else
	{
		m_t1 = p1->GetType();
		p1->SetType(e_staticParticle);

		m_t2 = p2->GetType();
		p2->SetType(e_staticParticle);

		m_t3 = p3->GetType();
		p3->SetType(e_staticParticle);
	}

	return true;
}

void b3ClothDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = GetPointA();
	b3Vec3 B = GetPointB();

	b3Vec3 dx = B - A;

	if (m_spring)
	{
		m_particle->SetPosition(B);
	}
	else
	{
		b3Particle* p1 = m_cloth->GetVertexParticle(m_triangle->v1);
		p1->ApplyTranslation(dx);

		b3Particle* p2 = m_cloth->GetVertexParticle(m_triangle->v2);
		p2->ApplyTranslation(dx);

		b3Particle* p3 = m_cloth->GetVertexParticle(m_triangle->v3);
		p3->ApplyTranslation(dx);
	}
}

void b3ClothDragger::SetSpring(bool bit)
{
	if (bit == m_spring)
	{
		return;
	}

	if (IsDragging())
	{
		StopDragging();
	}

	m_spring = bit;
}

void b3ClothDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);

	if (m_spring)
	{
		m_cloth->DestroyForce(m_s1);
		m_cloth->DestroyForce(m_s2);
		m_cloth->DestroyForce(m_s3);
		m_cloth->DestroyParticle(m_particle);
	}
	else
	{
		m_cloth->GetVertexParticle(m_triangle->v1)->SetType(m_t1);
		m_cloth->GetVertexParticle(m_triangle->v2)->SetType(m_t2);
		m_cloth->GetVertexParticle(m_triangle->v3)->SetType(m_t3);
	}

	m_triangle = nullptr;
}

b3Vec3 b3ClothDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 A = m_cloth->GetVertexParticle(m_triangle->v1)->GetPosition();
	b3Vec3 B = m_cloth->GetVertexParticle(m_triangle->v2)->GetPosition();
	b3Vec3 C = m_cloth->GetVertexParticle(m_triangle->v3)->GetPosition();

	return m_u * A + m_v * B + (1.0f - m_u - m_v) * C;
}

b3Vec3 b3ClothDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}