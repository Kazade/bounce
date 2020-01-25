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

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

class MotorTest : public Test
{
public:

	MotorTest()
	{
		b3Body* ground = nullptr;
		{
			// Ground
			b3BodyDef bd;
			ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			ground->CreateShape(sd);
		}

		{
			// Motorized body
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 8.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			m_boxHull.SetExtents(2.0f, 0.5f, 0.5f);

			b3HullShape shape;
			shape.m_hull = &m_boxHull;

			b3ShapeDef sd;
			sd.shape = &shape;
			sd.friction = 0.3f;
			sd.density = 2.0f;
			body->CreateShape(sd);

			b3MotorJointDef mjd;
			mjd.Initialize(ground, body);
			mjd.maxForce = 1000.0f;
			mjd.maxTorque = 1000.0f;
			m_joint = (b3MotorJoint*)m_world.CreateJoint(mjd);
		}

		m_play = false;
		m_x = 0.0f;
	}
	
	void KeyDown(int key)
	{
		if (key == GLFW_KEY_S)
		{
			m_play = !m_play;
		}
	}

	void Step()
	{
		if (m_play)
		{
			m_x += g_testSettings->inv_hertz;

			if (m_x >= 2.0f * B3_PI)
			{
				m_x = 0.0f;
			}
		}
		
		b3Vec3 linearOffset;
		linearOffset.x = 8.0f * sinf(2.0f * m_x);
		linearOffset.y = 8.0f + sinf(m_x);
		linearOffset.z = 0.0f;

		m_joint->SetLinearOffset(linearOffset);

		b3Quat angularOffset;
		angularOffset.SetAxisAngle(b3Vec3_z, m_x);
		angularOffset.Normalize();

		m_joint->SetAngularOffset(angularOffset);

		Test::Step();

		g_draw->DrawPoint(linearOffset, 4.0f, b3Color(0.9f, 0.9f, 0.9f));

		g_draw->DrawString(b3Color_white, "S - Play/Pause");
	}

	static Test* Create()
	{
		return new MotorTest();
	}
	
	b3BoxHull m_boxHull;
	b3MotorJoint* m_joint;
	bool m_play;
	scalar m_x;
};

#endif