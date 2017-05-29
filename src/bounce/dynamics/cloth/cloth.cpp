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

#include <bounce/dynamics/cloth/cloth.h>
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

			u32 v1 = is[j];
			u32 v2 = is[k];

			b3Vec3 p1 = m->vertices[v1];
			b3Vec3 p2 = m->vertices[v2];

			b3C1* C = m_c1s + m_c1Count;
			C->i1 = v1;
			C->i2 = v2;
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

	u32 c2Capacity = 0;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t1 = m->triangles + i;
		u32 i1s[3] = { t1->v1, t1->v2, t1->v3 };

		for (u32 j1 = 0; j1 < 3; ++j1)
		{
			u32 k1 = j1 + 1 < 3 ? j1 + 1 : 0;

			u32 t1v1 = i1s[j1];
			u32 t1v2 = i1s[k1];

			for (u32 j = i + 1; j < m->triangleCount; ++j)
			{
				b3Triangle* t2 = m->triangles + j;
				u32 i2s[3] = { t2->v1, t2->v2, t2->v3 };

				for (u32 j2 = 0; j2 < 3; ++j2)
				{
					u32 k2 = j2 + 1 < 3 ? j2 + 1 : 0;

					u32 t2v1 = i2s[j2];
					u32 t2v2 = i2s[k2];

					if (t1v1 == t2v2 && t1v2 == t2v1)
					{
						++c2Capacity;
					}
				}
			}
		}
	}

	m_c2Count = 0;
	m_c2s = (b3C2*)b3Alloc(c2Capacity * sizeof(b3C2));

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t1 = m->triangles + i;
		u32 i1s[3] = { t1->v1, t1->v2, t1->v3 };

		for (u32 j1 = 0; j1 < 3; ++j1)
		{
			u32 k1 = j1 + 1 < 3 ? j1 + 1 : 0;

			u32 t1v1 = i1s[j1];
			u32 t1v2 = i1s[k1];

			for (u32 j = i + 1; j < m->triangleCount; ++j)
			{
				b3Triangle* t2 = m->triangles + j;
				u32 i2s[3] = { t2->v1, t2->v2, t2->v3 };

				for (u32 j2 = 0; j2 < 3; ++j2)
				{
					u32 k2 = j2 + 1 < 3 ? j2 + 1 : 0;

					u32 t2v1 = i2s[j2];
					u32 t2v2 = i2s[k2];

					if (t1v1 == t2v2 && t1v2 == t2v1)
					{
						u32 k3 = k1 + 1 < 3 ? k1 + 1 : 0;
						u32 t1v3 = i1s[k3];

						u32 k4 = k2 + 1 < 3 ? k2 + 1 : 0;
						u32 t2v3 = i2s[k4];

						b3Vec3 p1 = m->vertices[t1v1];
						b3Vec3 p2 = m->vertices[t1v2];
						b3Vec3 p3 = m->vertices[t1v3];
						b3Vec3 p4 = m->vertices[t2v3];

						b3Vec3 n1 = b3Cross(p2 - p1, p3 - p1);
						n1.Normalize();
						
						b3Vec3 n2 = b3Cross(p2 - p1, p4 - p1);
						n2.Normalize();
						
						float32 x = b3Dot(n1, n2);

						b3Vec3 n3 = b3Cross(n1, n2);
						float32 y = b3Length(n3);

						b3C2* c = m_c2s + m_c2Count;
						c->i1 = t1v1;
						c->i2 = t1v2;
						c->i3 = t1v3;
						c->i4 = t2v3;
						c->angle = atan2(y, x);
						
						++m_c2Count;

						break;
					}
				}
			}
		}
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
		SolveC2();
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
	for (u32 i = 0; i < m_c1Count; ++i)
	{
		b3C1* c = m_c1s + i;
		
		b3Particle* p1 = m_ps + c->i1;
		b3Particle* p2 = m_ps + c->i2;

		float32 m1 = p1->im;
		float32 m2 = p2->im;

		float32 mass = m1 + m2;
		if (mass == 0.0f)
		{
			continue;
		}

		mass = 1.0f / mass;

		b3Vec3 J2 = p2->p - p1->p;
		float32 L = b3Length(J2);
		if (L > B3_EPSILON)
		{
			J2 /= L;
		}

		b3Vec3 J1 = -J2;

		float32 C = L - c->L;
		float32 impulse = -m_k1 * mass * C;

		p1->p += (m1 * impulse) * J1;
		p2->p += (m2 * impulse) * J2;
	}
}

void b3Cloth::SolveC2()
{
	for (u32 i = 0; i < m_c2Count; ++i)
	{
		b3C2* c = m_c2s + i;

		b3Particle* p1 = m_ps + c->i1;
		b3Particle* p2 = m_ps + c->i2;
		b3Particle* p3 = m_ps + c->i3;
		b3Particle* p4 = m_ps + c->i4;

		float32 m1 = p1->im;
		float32 m2 = p2->im;
		float32 m3 = p3->im;
		float32 m4 = p4->im;

		b3Vec3 v2 = p2->p - p1->p;
		b3Vec3 v3 = p3->p - p1->p;
		b3Vec3 v4 = p4->p - p1->p;
		
		b3Vec3 n1 = b3Cross(v2, v3);
		n1.Normalize();

		b3Vec3 n2 = b3Cross(v2, v4);
		n2.Normalize();

		float32 x = b3Dot(n1, n2);

		b3Vec3 J3 = b3Cross(v2, n2) + x * b3Cross(n1, v2);
		float32 L3 = b3Length(b3Cross(v2, v3));
		if (L3 > B3_EPSILON)
		{
			J3 /= L3;
		}
		
		b3Vec3 J4 = b3Cross(v2, n1) + x * b3Cross(n2, v2);
		float32 L4 = b3Length(b3Cross(v2, v4));
		if (L4 > B3_EPSILON)
		{
			J4 /= L4;
		}

		b3Vec3 J2_1 = b3Cross(v3, n2) + x * b3Cross(n1, v3);
		if (L3 > B3_EPSILON)
		{
			J2_1 /= L3;
		}

		b3Vec3 J2_2 = b3Cross(v4, n1) + x * b3Cross(n2, v4);
		if (L4 > B3_EPSILON)
		{
			J2_2 /= L4;
		}
		
		b3Vec3 J2 = -J2_1 - J2_2;
		
		b3Vec3 J1 = -J2 - J3 - J4;

		float32 mass = m1 * b3Dot(J1, J1) + m2 * b3Dot(J2, J2) + m3 * b3Dot(J3, J3) + m4 * b3Dot(J4, J4);
		if (mass == 0.0f)
		{
			continue;
		}

		mass = 1.0f / mass;
		
		b3Vec3 n3 = b3Cross(n1, n2);
		float32 y = b3Length(n3);

		float32 angle = atan2(y, x);
		float32 C = angle - c->angle;

		float32 impulse = -m_k2 * mass * y * C;

		p1->p += (m1 * impulse) * J1;
		p2->p += (m2 * impulse) * J2;
		p3->p += (m3 * impulse) * J3;
		p4->p += (m4 * impulse) * J4;
	}
}

void b3Cloth::Draw(b3Draw* draw) const
{
	b3Color color1(1.0f, 0.0f, 0.0f);
	b3Color color2(0.0f, 1.0f, 0.0f);
	b3Color color3(0.0f, 0.0f, 1.0f);
	b3Color color4(0.0f, 0.0f, 0.0f);

	const b3Mesh* m = m_mesh;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Particle* p1 = m_ps + t->v1;
		b3Particle* p2 = m_ps + t->v2;
		b3Particle* p3 = m_ps + t->v3;

		b3Vec3 v1 = p1->p;
		b3Vec3 v2 = p2->p;
		b3Vec3 v3 = p3->p;

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();

		b3Vec3 n2 = -n1;

		draw->DrawSolidTriangle(n1, v1, v2, v3, color3);
		draw->DrawSolidTriangle(n2, v1, v3, v2, color3);
	}
}