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

#ifndef SMASH_SOFTBODY_H
#define SMASH_SOFTBODY_H

#include <testbed/framework/softbody_dragger.h>

class SmashSoftBody : public Test
{
public:
	SmashSoftBody()
	{
		m_mesh.SetAsSphere(2.0f, 1);

		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			m_mesh.vertices[i].y += 3.0f;
		}

		// Create soft body
		b3SoftBodyDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.E = 100.0f;
		def.nu = 0.33f;

		m_body = new b3SoftBody(def);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);
		m_body->SetWorld(&m_world);

		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			b3SoftBodyNode* n = m_body->GetVertexNode(i);

			n->SetRadius(0.05f);
			n->SetFriction(0.2f);
		}

		// Create ground
		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			b3HullShape groundShape;
			groundShape.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &groundShape;
			sd.friction = 0.3f;

			b->CreateShape(sd);
		}

		// Create body
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.y = 10.0f;

			b3Body* b = m_world.CreateBody(bd);

			static b3BoxHull boxHull(5.0f, 1.0f, 5.0f);

			b3HullShape boxShape;
			boxShape.m_hull = &boxHull;

			b3ShapeDef sd;
			sd.shape = &boxShape;
			sd.density = 0.1f;
			sd.friction = 0.3f;

			b->CreateShape(sd);
		}
		
		m_bodyDragger = new b3SoftBodyDragger(&m_ray, m_body);
	}

	~SmashSoftBody()
	{
		delete m_bodyDragger;
		delete m_body;
	}

	void Step()
	{
		Test::Step();

		if (m_bodyDragger->IsDragging())
		{
			m_bodyDragger->Drag();
		}

		m_body->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_body->Draw();

		if (m_bodyDragger->IsDragging())
		{
			b3Vec3 pA = m_bodyDragger->GetPointA();
			b3Vec3 pB = m_bodyDragger->GetPointB();

			g_draw->DrawPoint(pA, 2.0f, b3Color_green);

			g_draw->DrawPoint(pB, 2.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_softBodySolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_softBodySolverIterations);

		float32 E = m_body->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_bodyDragger->IsDragging() == false)
		{
			m_bodyDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_bodyDragger->IsDragging() == true)
		{
			m_bodyDragger->StopDragging();
		}
	}

	static Test* Create()
	{
		return new SmashSoftBody();
	}

	b3QSoftBodyMesh m_mesh;

	b3SoftBody* m_body;
	b3SoftBodyDragger* m_bodyDragger;
};

#endif