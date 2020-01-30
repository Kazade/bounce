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

#ifndef MULTIPLE_PENDULUM
#define MULTIPLE_PENDULUM

class MultiplePendulum : public Test
{
public:
	MultiplePendulum()
	{
		b3Vec3 axis(0.0f, 0.0f, 1.0f);

		b3Body* bs[6];
		{
			b3BodyDef bd;
			bd.type = e_staticBody;
			bs[0] = m_world.CreateBody(bd);
		}
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-0.5f, 0.0f, 0.0f);
			bs[1] = m_world.CreateBody(bd);

			b3CapsuleShape s;
			s.m_vertex1.Set(0.5f, 0.0f, 0.0f);
			s.m_vertex2.Set(-0.5f, 0.0f, 0.0f);
			s.m_radius = 0.05f;

			b3ShapeDef sd;
			sd.shape = &s;
			sd.density = 10.0f;
			bs[1]->CreateShape(sd);

			b3RevoluteJointDef jd;
			jd.Initialize(bs[0], bs[1], axis, b3Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f);
			m_world.CreateJoint(jd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-1.5f, 0.0f, 0.0f);
			bs[2] = m_world.CreateBody(bd);

			b3CapsuleShape s;
			s.m_vertex1.Set(0.5f, 0.0f, 0.0f);
			s.m_vertex2.Set(-0.5f, 0.0f, 0.0f);
			s.m_radius = 0.05f;

			b3ShapeDef sd;
			sd.shape = &s;
			sd.density = 10.0f;
			bs[2]->CreateShape(sd);

			b3RevoluteJointDef jd;
			jd.Initialize(bs[1], bs[2], axis, b3Vec3(-1.0f, 0.0f, 0.0f), 0.0f, 1.0f);
			m_world.CreateJoint(jd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-2.5f, 0.0f, 0.0f);
			bs[3] = m_world.CreateBody(bd);

			b3CapsuleShape s;
			s.m_vertex1.Set(0.5f, 0.0f, 0.0f);
			s.m_vertex2.Set(-0.5f, 0.0f, 0.0f);
			s.m_radius = 0.05f;

			b3ShapeDef sd;
			sd.shape = &s;
			sd.density = 100.0f;
			bs[3]->CreateShape(sd);

			b3RevoluteJointDef jd;
			jd.Initialize(bs[2], bs[3], axis, b3Vec3(-2.0f, 0.0f, 0.0f), 0.0f, 1.0f);
			m_world.CreateJoint(jd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-3.5f, 0.0f, 0.0f);
			bs[4] = m_world.CreateBody(bd);

			b3CapsuleShape s;
			s.m_vertex1.Set(0.5f, 0.0f, 0.0f);
			s.m_vertex2.Set(-0.5f, 0.0f, 0.0f);
			s.m_radius = 0.05f;

			b3ShapeDef sd;
			sd.shape = &s;
			sd.density = 1000.0f;
			bs[4]->CreateShape(sd);

			b3RevoluteJointDef jd;
			jd.Initialize(bs[3], bs[4], axis, b3Vec3(-3.0f, 0.0f, 0.0f), 0.0f, 1.0f);
			m_world.CreateJoint(jd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-4.5f, 0.0f, 0.0f);
			bs[5] = m_world.CreateBody(bd);

			b3CapsuleShape s;
			s.m_vertex1.Set(0.5f, 0.0f, 0.0f);
			s.m_vertex2.Set(-0.5f, 0.0f, 0.0f);
			s.m_radius = 0.05f;

			b3ShapeDef sd;
			sd.shape = &s;
			sd.density = 50.0f;
			bs[5]->CreateShape(sd);

			b3RevoluteJointDef jd;
			jd.Initialize(bs[4], bs[5], axis, b3Vec3(-4.0f, 0.0f, 0.0f), 0.0f, 1.0f);
			m_world.CreateJoint(jd);
		}
	}

	static Test* Create()
	{
		return new MultiplePendulum();
	}
};

#endif