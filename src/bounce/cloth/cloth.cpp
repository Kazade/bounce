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

#include <bounce/cloth/cloth.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/common/template/array.h>
#include <bounce/common/draw.h>

b3Cloth::b3Cloth()
{
	m_pCount = 0;
	m_ps = NULL;
	m_c1Count = 0;
	m_c1s = NULL;
	m_c2Count = 0;
	m_c2s = NULL;

	m_k1 = 0.0f;
	m_k2 = 0.0f;
	m_kd = 0.0f;
	m_r = 0.0f;
	m_gravity.SetZero();
}

b3Cloth::~b3Cloth()
{
	b3Free(m_ps);
	b3Free(m_c1s);
	b3Free(m_c2s);
}

void b3Cloth::Initialize(const b3ClothDef& def)
{
	B3_ASSERT(def.mesh);
	m_mesh = def.mesh;
	
	const b3Mesh* m = m_mesh;

	m_pCount = m->vertexCount;
	m_ps = (b3Particle*)b3Alloc(m_pCount * sizeof(b3Particle));
	
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3Particle* p = m_ps + i;
		p->im = 0.0f;
		p->p = m->vertices[i];
		p->p0 = p->p;
		p->v.SetZero();
	}

	m_c1s = (b3C1*)b3Alloc(3 * m->triangleCount * sizeof(b3C1));
	m_c1Count = 0;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		u32 is[3] = { t->v1, t->v2, t->v3 };
		for (u32 j = 0; j < 3; ++j)
		{
			u32 k = j + 1 < 3 ? j + 1 : 0;

			u32 i1 = is[j];
			u32 i2 = is[k];

			b3Vec3 p1 = m->vertices[i1];
			b3Vec3 p2 = m->vertices[i2];

			b3C1* C = m_c1s + m_c1Count;
			C->i1 = i1;
			C->i2 = i2;
			C->L = b3Distance(p1, p2);
			++m_c1Count;
		}

		b3Vec3 p1 = m->vertices[t->v1];
		b3Vec3 p2 = m->vertices[t->v2];
		b3Vec3 p3 = m->vertices[t->v3];

		float32 area = b3Area(p1, p2, p3);
		float32 mass = def.density * area;

		const float32 inv3 = 1.0f / 3.0f;

		m_ps[t->v1].im += inv3 * mass;
		m_ps[t->v2].im += inv3 * mass;
		m_ps[t->v3].im += inv3 * mass;
	}

	for (u32 i = 0; i < m_pCount; ++i)
	{
		m_ps[i].im = m_ps[i].im > 0.0f ? 1.0f / m_ps[i].im : 0.0f;
	}

	m_k1 = def.k1;
	m_k2 = def.k2;
	m_kd = def.kd;
	m_r = def.r;
	m_gravity = def.gravity;
}

void b3Cloth::Step(float32 h, u32 iterations)
{
	if (h == 0.0f)
	{
		return;
	}

	float32 d = exp(-h * m_kd);

	for (u32 i = 0; i < m_pCount; ++i)
	{
		b3Particle* p = m_ps + i;

		p->v += h * p->im * m_gravity;
		p->v *= d;

		p->p0 = p->p;
		p->p += h * p->v;
	}

	for (u32 i = 0; i < iterations; ++i)
	{
		SolveC1();
	}

	float32 inv_h = 1.0f / h;
	for (u32 i = 0; i < m_pCount; ++i)
	{
		b3Particle* p = m_ps + i;
		p->v = inv_h * (p->p - p->p0);
	}
}

void b3Cloth::SolveC1()
{
	float32 k = m_k1;

	for (u32 i = 0; i < m_c1Count; ++i)
	{
		b3C1* c = m_c1s + i;
		
		b3Particle* p1 = m_ps + c->i1;
		b3Particle* p2 = m_ps + c->i2;

		b3Vec3 d = p2->p - p1->p;
		float32 L = b3Length(d);
		if (L > B3_EPSILON)
		{
			d /= L;
		}

		float32 C = L - c->L;
		
		float32 im1 = p1->im;
		float32 im2 = p2->im;

		if (im1 + im2 == 0.0f)
		{
			continue;
		}

		float32 s1 = im1 / (im1 + im2);
		float32 s2 = im2 / (im1 + im2);

		p1->p -= k * s1 * -C * d;
		p2->p += k * s2 * -C * d;
	}
}

void b3Cloth::Draw(b3Draw* draw) const
{
	const b3Color color1(1.0f, 0.0f, 0.0f);
	const b3Color color2(0.0f, 1.0f, 0.0f);
	const b3Color color3(0.0f, 0.0f, 1.0f);
	const b3Color color4(0.0f, 0.0f, 0.0f);

	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3Triangle* t = m_mesh->triangles + i;

		b3Particle* p1 = m_ps + t->v1;
		b3Particle* p2 = m_ps + t->v2;
		b3Particle* p3 = m_ps + t->v3;

		b3Vec3 ps[3];
		ps[0] = p1->p;
		ps[1] = p2->p;
		ps[2] = p3->p;

		draw->DrawPolygon(ps, 3, color4);
		draw->DrawSolidPolygon(ps, 3, color3);
	}
}