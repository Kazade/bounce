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

b3ClothDragger::b3ClothDragger(b3Ray3* ray, b3Cloth* cloth)
{
	m_staticDrag = true;
	m_ray = ray;
	m_cloth = cloth;
	m_isDragging = false;
	m_km = 100000.0f;
	m_kd = 1000.0f;
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

	m_isDragging = true;
	m_x = rayOut.fraction;

	m_p1 = rayOut.triangle->GetParticle1();
	m_p2 = rayOut.triangle->GetParticle2();
	m_p3 = rayOut.triangle->GetParticle3();

	b3Vec3 v1 = m_p1->GetPosition();
	b3Vec3 v2 = m_p2->GetPosition();
	b3Vec3 v3 = m_p3->GetPosition();

	b3Vec3 B = GetPointB();

	scalar wABC[4];
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

	if (m_staticDrag)
	{
		m_t1 = m_p1->GetType();
		m_p1->SetType(e_staticClothParticle);

		m_t2 = m_p2->GetType();
		m_p2->SetType(e_staticClothParticle);

		m_t3 = m_p3->GetType();
		m_p3->SetType(e_staticClothParticle);
	}
	else
	{
		b3ClothParticleDef pd;
		pd.type = e_staticClothParticle;
		pd.position = GetPointA();

		m_particle = m_cloth->CreateParticle(pd);

		b3MouseForceDef def;
		def.p1 = m_particle;
		def.p2 = m_p1;
		def.p3 = m_p2;
		def.p4 = m_p3;
		def.w2 = m_u;
		def.w3 = m_v;
		def.w4 = 1.0f - m_u - m_v;
		def.mouse = m_km;
		def.damping = m_kd;
		def.restLength = 0.0f;

		m_mf = (b3MouseForce*)m_cloth->CreateForce(def);
	}

	return true;
}

void b3ClothDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 B = GetPointB();

	if (m_staticDrag)
	{
		b3Vec3 A = GetPointA();

		b3Vec3 dx = B - A;

		m_p1->ApplyTranslation(dx);
		m_p2->ApplyTranslation(dx);
		m_p3->ApplyTranslation(dx);
	}
	else
	{
		//b3Vec3 A = m_particle->GetPosition();
		//b3Vec3 dx = B - A;		
		//m_particle->ApplyTranslation(dx);
		m_particle->SetPosition(B);
	}
}

void b3ClothDragger::SetStaticDrag(bool bit)
{
	if (bit == m_staticDrag)
	{
		return;
	}

	if (IsDragging())
	{
		StopDragging();
	}

	m_staticDrag = bit;
}

void b3ClothDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);

	if (m_staticDrag)
	{
		m_p1->SetType(m_t1);
		m_p2->SetType(m_t2);
		m_p3->SetType(m_t3);
	}
	else
	{
		m_cloth->DestroyForce(m_mf);
		m_cloth->DestroyParticle(m_particle);
	}

	m_isDragging = false;
}

b3Vec3 b3ClothDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 v1 = m_p1->GetPosition() + m_p1->GetTranslation();
	b3Vec3 v2 = m_p2->GetPosition() + m_p2->GetTranslation();
	b3Vec3 v3 = m_p3->GetPosition() + m_p3->GetTranslation();

	return m_u * v1 + m_v * v2 + (1.0f - m_u - m_v) * v3;
}

b3Vec3 b3ClothDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}