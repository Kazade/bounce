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

#ifndef TENSION_MAPPING_H
#define TENSION_MAPPING_H

// Hot/Cold color map
// See http://paulbourke.net/miscellaneous/colourspace/
static inline b3Color Color(scalar x, scalar a, scalar b)
{
	x = b3Clamp(x, a, b);

	scalar d = b - a;

	b3Color c(1.0f, 1.0f, 1.0f); 

	if (x < a + 0.25f * d) 
	{
		c.r = 0.0f;
		c.g = 4.0f * (x - a) / d;
		return c;
	}
	
	if (x < a + 0.5f * d) 
	{
		c.r = 0.0f;
		c.b = 1.0f + 4.0f * (a + 0.25f * d - x) / d;
		return c;
	}

	if (x < a + 0.75f * d) 
	{
		c.r = 4.0f * (x - a - 0.5f * d) / d;
		c.b = 0.0f;
		return c;
	}

	c.g = 1.0f + 4.0f * (a + 0.75f * d - x) / d;
	c.b = 0.0f;
	return c;
}

class TensionMapping : public Test
{
public:
	enum
	{
		e_w = 10,
		e_h = 10
	};

	TensionMapping()
	{
		// Create cloth
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.streching = 10000.0f;
		def.strechDamping = 100.0f;
		def.shearing = 1000.0f;
		def.shearDamping = 10.0f;
		def.bending = 1000.0f;
		def.bendDamping = 10.0f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		// Freeze some particles
		for (u32 i = 0; i < 2; ++i)
		{
			for (u32 j = 0; j < e_w + 1; ++j)
			{
				u32 v = m_clothMesh.GetVertex(i, j);
				
				b3ClothParticle* p = m_cloth->GetParticle(v);
				p->SetType(e_staticClothParticle);
			}
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~TensionMapping()
	{
		delete m_clothDragger;
		delete m_cloth;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		const b3ClothMesh* mesh = m_cloth->GetMesh();

		b3Vec3 tension[(e_h + 1) * (e_w + 1)];
		for (u32 i = 0; i < mesh->vertexCount; ++i)
		{
			tension[i].SetZero();
		}

		for (b3Force* f = m_cloth->GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() == e_stretchForce)
			{
				b3StretchForce* s = (b3StretchForce*)f;

				b3Vec3 f1 = s->GetActionForce1();
				b3Vec3 f2 = s->GetActionForce2();
				b3Vec3 f3 = s->GetActionForce3();

				b3ClothParticle* p1 = s->GetParticle1();
				b3ClothParticle* p2 = s->GetParticle2();
				b3ClothParticle* p3 = s->GetParticle3();

				u32 v1 = p1->GetMeshIndex();
				u32 v2 = p2->GetMeshIndex();
				u32 v3 = p3->GetMeshIndex();

				tension[v1] += f1;
				tension[v2] += f2;
				tension[v3] += f3;
			}
		}
		
		for (b3ClothParticle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
			if (p->GetType() == e_staticClothParticle)
			{
				b3Draw_draw->DrawPoint(p->GetPosition(), 4.0f, b3Color_white);
			}

			if (p->GetType() == e_kinematicClothParticle)
			{
				b3Draw_draw->DrawPoint(p->GetPosition(), 4.0f, b3Color_blue);
			}

			if (p->GetType() == e_dynamicClothParticle)
			{
				b3Draw_draw->DrawPoint(p->GetPosition(), 4.0f, b3Color_green);
			}
		}

		for (u32 i = 0; i < mesh->triangleCount; ++i)
		{
			b3ClothMeshTriangle* triangle = mesh->triangles + i;

			b3Vec3 v1 = m_cloth->GetParticle(triangle->v1)->GetPosition();
			b3Vec3 v2 = m_cloth->GetParticle(triangle->v2)->GetPosition();
			b3Vec3 v3 = m_cloth->GetParticle(triangle->v3)->GetPosition();

			g_draw->DrawTriangle(v1, v2, v3, b3Color_black);

			b3Vec3 c = (v1 + v2 + v3) / 3.0f;

			scalar s = 0.9f;

			v1 = s * (v1 - c) + c;
			v2 = s * (v2 - c) + c;
			v3 = s * (v3 - c) + c;

			b3Vec3 f1 = tension[triangle->v1];
			scalar L1 = b3Length(f1);

			b3Vec3 f2 = tension[triangle->v2];
			scalar L2 = b3Length(f2);

			b3Vec3 f3 = tension[triangle->v3];
			scalar L3 = b3Length(f3);

			scalar L = (L1 + L2 + L3) / 3.0f;

			const scalar kMaxT = 10000.0f;
			b3Color color = Color(L, 0.0f, kMaxT);
			
			b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
			n1.Normalize();

			scalar r = 0.05f;

			{
				b3Vec3 x1 = v1 + r * n1;
				b3Vec3 x2 = v2 + r * n1;
				b3Vec3 x3 = v3 + r * n1;

				g_draw->DrawSolidTriangle(n1, x1, x2, x3, color);
			}

			{
				b3Vec3 n2 = -n1;

				b3Vec3 x1 = v1 + r * n2;
				b3Vec3 x2 = v2 + r * n2;
				b3Vec3 x3 = v3 + r * n2;

				g_draw->DrawSolidTriangle(n2, x3, x2, x1, color);
			}
		}

		if (m_clothDragger->IsDragging())
		{
			b3Vec3 pA = m_clothDragger->GetPointA();
			b3Vec3 pB = m_clothDragger->GetPointB();

			g_draw->DrawPoint(pA, 4.0f, b3Color_green);

			g_draw->DrawPoint(pB, 4.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_clothSolverIterations);

		scalar E = m_cloth->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->Drag();
		}
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_clothDragger->IsDragging() == false)
		{
			m_clothDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->StopDragging();
		}
	}

	static Test* Create()
	{
		return new TensionMapping();
	}

	b3GridClothMesh<e_w, e_h> m_clothMesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger; 
};

#endif