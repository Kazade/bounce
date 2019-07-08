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

#ifndef BODY_TYPES_H
#define BODY_TYPES_H

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			ground->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 3.5f, 0.0f);
			bd.linearVelocity.Set(0.0f, 0.0f, 0.0f);
			bd.angularVelocity.Set(0.0f, B3_PI, 0.0f);

			m_body = m_world.CreateBody(bd);

			b3CapsuleShape cap;
			cap.m_centers[0].Set(0.0f, 2.0f, 0.0f);
			cap.m_centers[1].Set(0.0f, -2.0f, 0.0f);
			cap.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cap;
			sd.density = 1.5f;
			sd.friction = 0.7f;

			m_body->CreateShape(sd);
		}
	}

	void Step()
	{
		Test::Step();

		g_draw->DrawString(b3Color_white, "S - Static");
		g_draw->DrawString(b3Color_white, "D - Dynamic");
		g_draw->DrawString(b3Color_white, "K - Kinematic");
		g_draw->DrawString(b3Color_white, "Space - Throw Bomb");
		g_draw->DrawString(b3Color_white, "Arrows - Apply Force/Velocity/Position");
	}

	void KeyDown(int button) 
	{
		if (button == GLFW_KEY_S)
		{
			m_body->SetType(e_staticBody);
		}
		
		if (button == GLFW_KEY_K)
		{
			m_body->SetType(e_kinematicBody);
		}

		if (button == GLFW_KEY_D)
		{
			m_body->SetType(e_dynamicBody);
		}

		if (button == GLFW_KEY_SPACE)
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(RandomFloat(-20.0f, 20.0f), RandomFloat(10.0f, 20.0f), RandomFloat(-20.0f, 20.0f));

			b3Vec3 n = m_body->GetTransform().position - bd.position;
			n.Normalize();

			bd.linearVelocity = 100.0f * n;

			b3Body* enemy = m_world.CreateBody(bd);

			b3SphereShape shape;
			shape.m_center.Set(0.0f, 0.0f, 0.0f);
			shape.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &shape;
			sd.density = 1.0f;
			sd.friction = 1.0f;

			enemy->CreateShape(sd);
		}

		if (m_body->GetType() == e_staticBody)
		{
			if (button == GLFW_KEY_LEFT)
			{
				b3Vec3 p = m_body->GetSweep().worldCenter;
				b3Quat q = m_body->GetSweep().orientation;
				
				p.x -= 1.0f;

				m_body->SetTransform(p, b3Vec3(q.x, q.y, q.z), q.w);
			}

			if (button == GLFW_KEY_RIGHT)
			{
				b3Vec3 p = m_body->GetSweep().worldCenter;
				b3Quat q = m_body->GetSweep().orientation;

				p.x += 1.0f;

				m_body->SetTransform(p, b3Vec3(q.x, q.y, q.z), q.w);
			}

			if (button == GLFW_KEY_UP)
			{
				b3Vec3 p = m_body->GetSweep().worldCenter;
				b3Quat q = m_body->GetSweep().orientation;

				p.z += 1.0f;

				m_body->SetTransform(p, b3Vec3(q.x, q.y, q.z), q.w);
			}

			if (button == GLFW_KEY_DOWN)
			{
				b3Vec3 p = m_body->GetSweep().worldCenter;
				b3Quat q = m_body->GetSweep().orientation;

				p.z -= 1.0f;

				m_body->SetTransform(p, b3Vec3(q.x, q.y, q.z), q.w);
			}
		}

		if (m_body->GetType() == e_kinematicBody)
		{
			if (button == GLFW_KEY_LEFT)
			{
				b3Vec3 v = m_body->GetLinearVelocity();
				b3Vec3 w = m_body->GetAngularVelocity();

				v.x -= 5.0f;
				w.y -= 0.25f * B3_PI;

				m_body->SetLinearVelocity(v);
				m_body->SetAngularVelocity(w);
			}

			if (button == GLFW_KEY_RIGHT)
			{
				b3Vec3 v = m_body->GetLinearVelocity();
				b3Vec3 w = m_body->GetAngularVelocity();

				v.x += 5.0f;
				w.y += 0.25f * B3_PI;

				m_body->SetLinearVelocity(v);
				m_body->SetAngularVelocity(w);
			}

			if (button == GLFW_KEY_UP)
			{
				b3Vec3 v = m_body->GetLinearVelocity();
				b3Vec3 w = m_body->GetAngularVelocity();

				v.z -= 5.0f;
				w.y -= 0.25f * B3_PI;

				m_body->SetLinearVelocity(v);
				m_body->SetAngularVelocity(w);
			}

			if (button == GLFW_KEY_DOWN)
			{
				b3Vec3 v = m_body->GetLinearVelocity();
				b3Vec3 w = m_body->GetAngularVelocity();

				v.z += 5.0f;
				w.y += 0.25f * B3_PI;

				m_body->SetLinearVelocity(v);
				m_body->SetAngularVelocity(w);
			}
		}

		if (m_body->GetType() == e_dynamicBody)
		{
			if (button == GLFW_KEY_LEFT)
			{
				m_body->ApplyForceToCenter(b3Vec3(-100.0f, 0.0f, 0.0f), true);
			}

			if (button == GLFW_KEY_RIGHT)
			{
				m_body->ApplyForceToCenter(b3Vec3(100.0f, 0.0f, 0.0f), true);
			}

			if (button == GLFW_KEY_UP)
			{
				m_body->ApplyForceToCenter(b3Vec3(0.0f, 0.0f, -100.0f), true);
			}

			if (button == GLFW_KEY_DOWN)
			{
				m_body->ApplyForceToCenter(b3Vec3(0.0f, 0.0f, 100.0f), true);
			}
		}
	}

	static Test* Create()
	{
		return new BodyTypes();
	}

	b3Body* m_body;
};

#endif
