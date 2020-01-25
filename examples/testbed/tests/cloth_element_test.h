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

#ifndef CLOTH_ELEMENT_TEST_H
#define CLOTH_ELEMENT_TEST_H

class ClothElementTest : public Test
{
public:
	enum
	{
		e_w = 10,
		e_h = 10
	};

	ClothElementTest()
	{
		g_camera->m_zoom = 20.0f;

		b3GridClothMesh<e_w, e_h> m;

		b3ClothParticle** particles = (b3ClothParticle * *)b3Alloc(m.vertexCount * sizeof(b3ClothParticle*));
		for (u32 i = 0; i < m.vertexCount; ++i)
		{
			b3ClothParticleDef pd;
			pd.type = e_dynamicClothParticle;
			pd.position = m.vertices[i];

			b3ClothParticle* p = m_cloth.CreateParticle(pd);
			particles[i] = p;

			b3ClothSphereShapeDef sd;
			sd.p = p;
			sd.radius = 0.2f;
			sd.friction = 0.4f;

			m_cloth.CreateSphereShape(sd);
		}

		for (u32 i = 0; i < m.triangleCount; ++i)
		{
			u32 v1 = m.triangles[i].v1;
			u32 v2 = m.triangles[i].v2;
			u32 v3 = m.triangles[i].v3;

			b3Vec3 x1 = m.vertices[v1];
			b3Vec3 x2 = m.vertices[v2];
			b3Vec3 x3 = m.vertices[v3];

			b3ClothParticle* p1 = particles[v1];
			b3ClothParticle* p2 = particles[v2];
			b3ClothParticle* p3 = particles[v3];

			b3ClothTriangleShapeDef tsd;
			tsd.p1 = p1;
			tsd.p2 = p2;
			tsd.p3 = p3;
			tsd.v1 = x1;
			tsd.v2 = x2;
			tsd.v3 = x3;
			tsd.density = 0.1f;

			m_cloth.CreateTriangleShape(tsd);

			b3ElementForceDef fd;
			fd.p1 = p1;
			fd.p2 = p2;
			fd.p3 = p3;
			fd.E_x = 500.0f;
			fd.E_y = 500.0f;
			fd.E_s = 500.0f;
			fd.nu_xy = 0.3f;
			fd.nu_yx = 0.3f;
			fd.v1 = x1;
			fd.v2 = x2;
			fd.v3 = x3;

			m_cloth.CreateForce(fd);
		}

		for (u32 i = 0; i < e_w + 1; ++i)
		{
			u32 vertex = m.GetVertex(0, i);
			particles[vertex]->SetType(e_staticClothParticle);
		}

		b3Free(particles);

		m_cloth.SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_clothDragger = new b3ClothDragger(&m_ray, &m_cloth);
		m_clothDragger->SetStaticDrag(false);
	}

	~ClothElementTest()
	{
		delete m_clothDragger;
	}

	void Step()
	{
		Test::Step();

		m_cloth.Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_cloth.Draw();

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

		scalar E = m_cloth.GetEnergy();
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
		return new ClothElementTest();
	}

	b3Cloth m_cloth;
	b3ClothDragger* m_clothDragger;
};

#endif