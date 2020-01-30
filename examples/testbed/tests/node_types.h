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

#ifndef NODE_TYPES_H
#define NODE_TYPES_H

class NodeTypes : public Test
{
public:
	enum
	{
		e_w = 5,
		e_h = 2,
		e_d = 2
	};

	NodeTypes()
	{
		// Create soft body
		b3SoftBodyDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.E = 1000.0f;
		def.nu = 0.33f;
		def.radius = 0.2f;
		def.friction = 0.6f;

		m_body = new b3SoftBody(def);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		for (u32 i = 0; i < e_h + 1; ++i)
		{
			for (u32 k = 0; k < e_d + 1; ++k)
			{
				u32 v = m_mesh.GetVertex(i, 0, k);

				b3SoftBodyNode* n = m_body->GetNode(v);
				n->SetType(e_staticSoftBodyNode);
			}
		}

		m_bodyDragger = new b3SoftBodyDragger(&m_ray, m_body);
	}

	~NodeTypes()
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

		g_draw->DrawString(b3Color_white, "S - Static");
		g_draw->DrawString(b3Color_white, "D - Dynamic");
		g_draw->DrawString(b3Color_white, "K - Kinematic");
		g_draw->DrawString(b3Color_white, "Arrows - Apply Force/Velocity/Position");
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

	void SetBodyType(b3SoftBodyNodeType type)
	{
		for (u32 i = 0; i < e_h + 1; ++i)
		{
			for (u32 k = 0; k < e_d + 1; ++k)
			{
				u32 v = m_mesh.GetVertex(i, 0, k);

				b3SoftBodyNode* n = m_body->GetNode(v);
				n->SetType(type);
			}
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetBodyType(e_staticSoftBodyNode);
		}

		if (button == GLFW_KEY_K)
		{
			SetBodyType(e_kinematicSoftBodyNode);
		}

		if (button == GLFW_KEY_D)
		{
			SetBodyType(e_dynamicSoftBodyNode);
		}

		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			b3SoftBodyNode* n = m_body->GetNode(i);

			b3Vec3 d;
			d.SetZero();

			if (button == GLFW_KEY_LEFT)
			{
				d.x = -1.0f;
			}

			if (button == GLFW_KEY_RIGHT)
			{
				d.x = 1.0f;
			}

			if (button == GLFW_KEY_UP)
			{
				d.y = 1.0f;
			}

			if (button == GLFW_KEY_DOWN)
			{
				d.y = -1.0f;
			}

			if (button == GLFW_KEY_LEFT ||
				button == GLFW_KEY_RIGHT ||
				button == GLFW_KEY_UP ||
				button == GLFW_KEY_DOWN)
			{
				if (n->GetType() == e_staticSoftBodyNode)
				{
					n->ApplyTranslation(d);
				}

				if (n->GetType() == e_kinematicSoftBodyNode)
				{
					b3Vec3 v = n->GetVelocity();

					v += 5.0f * d;

					n->SetVelocity(v);
				}

				if (n->GetType() == e_dynamicSoftBodyNode)
				{
					b3Vec3 f = 100.0f * d;

					n->ApplyForce(f);
				}
			}
		}
	}

	static Test* Create()
	{
		return new NodeTypes();
	}

	b3BlockSoftBodyMesh<e_w, e_h, e_d> m_mesh;
	b3SoftBody* m_body;
	b3SoftBodyDragger* m_bodyDragger;
};

#endif