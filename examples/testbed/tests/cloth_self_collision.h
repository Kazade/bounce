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

#ifndef CLOTH_SELF_COLLISION_H
#define CLOTH_SELF_COLLISION_H

class ClothSelfCollision : public Test
{
public:
	ClothSelfCollision()
	{
		// Translate the mesh
		for (u32 i = 0; i < m_clothMesh.vertexCount; ++i)
		{
			m_clothMesh.vertices[i].y += 5.0f;
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 1.0f;
		def.streching = 100000.0f;
		def.thickness = 0.2f;
		def.friction = 0.3f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));
		m_cloth->SetWorld(&m_world);

		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			b3CapsuleShape capsuleShape;
			capsuleShape.m_centers[0].Set(0.0f, 0.0f, -5.0f);
			capsuleShape.m_centers[1].Set(0.0f, 0.0f, 5.0f);
			capsuleShape.m_radius = 1.0f;;

			b3ShapeDef sd;
			sd.shape = &capsuleShape;
			sd.friction = 1.0f;

			b->CreateShape(sd);
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~ClothSelfCollision()
	{
		delete m_cloth;
		delete m_clothDragger;
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
		return new ClothSelfCollision();
	}

	b3GridClothMesh<10, 10> m_clothMesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger; 
};

#endif