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

#ifndef SOFTBODY_ANCHOR_H
#define SOFTBODY_ANCHOR_H

class SoftBodyAnchor : public Test
{
public:
	SoftBodyAnchor()
	{
		m_mesh.SetAsSphere(4.0f, 0);

		// Create soft body
		b3SoftBodyDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.E = 1000.0f;
		def.nu = 0.33f;
		def.c_yield = 0.1f;
		def.c_creep = 0.5f;
		def.c_max = 1.0f;
		def.massDamping = 0.2f;

		m_body = new b3SoftBody(def);

		u32 pinIndex = ~0;
		scalar pinDot = -B3_MAX_SCALAR;
		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			scalar dot = b3Dot(m_mesh.vertices[i], b3Vec3_y);
			if (dot > pinDot)
			{
				pinDot = dot;
				pinIndex = i;
			}
		}

		b3SoftBodyNode* pinNode = m_body->GetNode(pinIndex);
		pinNode->SetType(e_staticSoftBodyNode);

		u32 anchorIndex = ~0;
		scalar anchorDot = -B3_MAX_SCALAR;
		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			scalar dot = b3Dot(m_mesh.vertices[i], -b3Vec3_y);
			if (dot > anchorDot)
			{
				anchorDot = dot;
				anchorIndex = i;
			}
		}
		
		b3SoftBodyNode* anchorNode = m_body->GetNode(anchorIndex);

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.y = -10.0f;
			
			b3Body* b = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_vertex1.Set(0.0f, -1.0f, 0.0f);
			cs.m_vertex2.Set(0.0f, 1.0f, 0.0f);
			cs.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 0.1f;

			m_shape = b->CreateShape(sd);

			// Create anchor
			b3SoftBodyAnchorDef ad;
			ad.Initialize(b, anchorNode, anchorNode->GetPosition());
			
			m_body->CreateAnchor(ad);
		}

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		m_bodyDragger = new b3SoftBodyDragger(&m_ray, m_body);
	}

	~SoftBodyAnchor()
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

			g_draw->DrawPoint(pA, 4.0f, b3Color_green);

			g_draw->DrawPoint(pB, 4.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_softBodySolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_softBodySolverIterations);

		scalar E = m_body->GetEnergy();
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
		return new SoftBodyAnchor();
	}

	b3QSoftBodyMesh m_mesh;

	b3SoftBody* m_body;
	b3SoftBodyDragger* m_bodyDragger;
	b3Shape* m_shape;
};

#endif