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

#ifndef PINNED_CLOTH_H
#define PINNED_CLOTH_H

class PinnedCloth : public Test
{
public:
	PinnedCloth() : m_rectangleGarment(5.0f, 5.0f)
	{
		// Generate 2D mesh
		m_rectangleGarmentMesh.Set(&m_rectangleGarment, 1.0f);

		// Create 3D mesh
		m_rectangleClothMesh.Set(&m_rectangleGarmentMesh);
		
		// Rotate the mesh
		b3Mat33 rotation = b3Mat33RotationX(0.5f * B3_PI);
		for (u32 i = 0; i < m_rectangleClothMesh.vertexCount; ++i)
		{
			m_rectangleClothMesh.vertices[i] = rotation * m_rectangleClothMesh.vertices[i];
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_rectangleClothMesh;
		def.density = 0.2f;
		def.structural = 100000.0f;
		def.damping = 0.0f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));
		m_cloth->SetWorld(&m_world);

		// Freeze some particles
		b3AABB3 aabb1;
		aabb1.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb1.m_upper.Set(5.0f, 1.0f, -4.0f);

		b3AABB3 aabb2;
		aabb2.m_lower.Set(-5.0f, -1.0f, 4.0f);
		aabb2.m_upper.Set(5.0f, 1.0f, 6.0f);

		for (b3Particle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
			if (aabb1.Contains(p->GetPosition()))
			{
				p->SetType(e_staticParticle);
			}

			if (aabb2.Contains(p->GetPosition()))
			{
				p->SetType(e_staticParticle);
			}
		}
		
		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~PinnedCloth()
	{
		delete m_clothDragger;
		delete m_cloth;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_cloth->Draw();

		if (m_clothDragger->IsDragging())
		{
			b3Vec3 pA = m_clothDragger->GetPointA();
			b3Vec3 pB = m_clothDragger->GetPointB();

			g_draw->DrawPoint(pA, 2.0f, b3Color_green);

			g_draw->DrawPoint(pB, 2.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_clothSolverIterations);

		float32 E = m_cloth->GetEnergy();
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
		return new PinnedCloth();
	}

	b3RectangleGarment m_rectangleGarment;
	b3GarmentMesh m_rectangleGarmentMesh;
	b3GarmentClothMesh m_rectangleClothMesh;

	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger;
};

#endif