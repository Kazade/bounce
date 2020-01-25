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

#ifndef CAPE_H
#define CAPE_H

class Cape : public Test
{
public:
	enum
	{
		e_w = 5,
		e_h = 10
	};

	Cape()
	{
		// Translate the cloth mesh
		for (u32 i = 0; i < m_clothMesh.vertexCount; ++i)
		{
			m_clothMesh.vertices[i].y += 5.0f;
			m_clothMesh.vertices[i].z -= 6.0f;
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.streching = 100000.0f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		// Freeze some particles
		for (u32 j = 0; j < e_w + 1; ++j)
		{
			u32 vj = m_clothMesh.GetVertex(e_h, j);

			b3ClothParticle* p = m_cloth->GetParticle(vj);
			p->SetType(e_kinematicClothParticle);
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);

		{
			// Create body
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_kinematicBody;

			m_body = m_world.CreateBody(bdef);

			static b3BoxHull box(1.0f, 5.0f, 1.0f);
			
			b3HullShape hs;
			hs.m_hull = &box;
			hs.m_radius = 0.25f;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
			sdef.friction = 0.3f;
			sdef.shape = &hs;

			m_body->CreateShape(sdef);
		}

		// Store cloth vertices in body space
		for (u32 j = 0; j < e_w + 1; ++j)
		{
			u32 vj = m_clothMesh.GetVertex(e_h, j);

			b3ClothParticle* p = m_cloth->GetParticle(vj);
			b3Vec3 position = p->GetPosition();
			
			m_localPoints[j] = m_body->GetLocalPoint(position);
		}
	}

	~Cape()
	{
		delete m_clothDragger;
		delete m_cloth;
	}
	
	void KeyDown(int button)
	{
		b3Vec3 v = m_body->GetLinearVelocity();
		
		if (button == GLFW_KEY_LEFT)
		{
			v.x -= 5.0f;
		}

		if (button == GLFW_KEY_RIGHT)
		{
			v.x += 5.0f;
		}

		if (button == GLFW_KEY_UP)
		{
			v.z -= 5.0f;
		}

		if (button == GLFW_KEY_DOWN)
		{
			v.z += 5.0f;
		}
		
		m_body->SetLinearVelocity(v);
	}

	void Step()
	{
		Test::Step();

		scalar inv_h = g_testSettings->hertz;

		for (u32 j = 0; j < e_w + 1; ++j)
		{
			u32 vj = m_clothMesh.GetVertex(e_h, j);

			b3ClothParticle* p = m_cloth->GetParticle(vj);
			b3Vec3 x0 = p->GetPosition();

			b3Vec3 x = m_body->GetWorldPoint(m_localPoints[j]);

			// Apply finite difference method
			b3Vec3 v = inv_h * (x - x0);

			p->SetVelocity(v);
		}

		m_cloth->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_cloth->Draw();

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
		
		g_draw->DrawString(b3Color_white, "Arrows - Apply Velocity");
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
		return new Cape();
	}

	b3GridClothMesh<e_w, e_h> m_clothMesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger;
	b3Body* m_body;
	b3Vec3 m_localPoints[e_w + 1];
};

#endif