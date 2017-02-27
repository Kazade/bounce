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

#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

class SensorTest : public Test
{
public:
	SensorTest()
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
			bd.position.Set(0.0f, 6.0f, 0.0f);
			
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_tallHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.isSensor = true;
			m_sensor = body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 4.0f, 10.0f);
			bd.linearVelocity.Set(0.0f, 0.0f, -5.0f);

			m_character = m_world.CreateBody(bd);

			b3CapsuleShape cap;
			cap.m_centers[0].Set(0.0f, 2.0f, 0.0f);
			cap.m_centers[1].Set(0.0f, -2.0f, 0.0f);
			cap.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cap;
			sd.density = 1.5f;
			sd.friction = 0.7f;

			m_character->CreateShape(sd);
		}

		m_attack = false;
	}

	void BeginContact(b3Contact* c)
	{
		b3Shape* sA = c->GetShapeA();
		b3Body* bA = sA->GetBody();
		b3Shape* sB = c->GetShapeB();
		b3Body* bB = sB->GetBody();

		if (sA == m_sensor)
		{
			if (bB == m_character)
			{
				m_attack = true;
			}
		}

		if (sB == m_sensor)
		{
			if (bA == m_character)
			{
				m_attack = true;
			}
		}

	}

	void EndContact(b3Contact* c)
	{
		b3Shape* sA = c->GetShapeA();
		b3Body* bA = sA->GetBody();
		b3Shape* sB = c->GetShapeB();
		b3Body* bB = sB->GetBody();

		if (sA == m_sensor)
		{
		}

		if (sB == m_sensor)
		{
		}
	}

	void Step()
	{
		if (m_attack)
		{
			b3Body* sensorBody = m_sensor->GetBody();

			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(RandomFloat(-20.0f, 20.0f), RandomFloat(10.0f, 20.0f), RandomFloat(-20.0f, 20.0f));

			b3Vec3 n = m_character->GetTransform().position - bd.position;
			n.Normalize();

			bd.linearVelocity = 60.0f * n;

			b3Body* enemy = m_world.CreateBody(bd);

			b3SphereShape shape;
			shape.m_center.Set(0.0f, 0.0f, 0.0f);
			shape.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &shape;
			sd.density = 1.0f;
			sd.friction = 1.0f;

			enemy->CreateShape(sd);
			
			m_attack = false;
		}

		Test::Step();
	}

	static Test* Create()
	{
		return new SensorTest();
	}
	
	b3Body* m_character;
	b3Shape* m_sensor;
	bool m_attack;
};

#endif
