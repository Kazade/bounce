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

#ifndef PRISMATIC_TEST_H
#define PRISMATIC_TEST_H

class PrismaticTest : public Test
{
public:
	PrismaticTest()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape shape;
			shape.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &shape;

			ground->CreateShape(sd);
		}

		b3Body* bA, * bB;

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-5.0f, 5.0f, 0.0f);

			bA = m_world.CreateBody(bd);

			m_hullA.SetExtents(2.0f, 2.0f, 0.5f);

			b3HullShape hull;
			hull.m_hull = &m_hullA;

			b3ShapeDef sdef;
			sdef.shape = &hull;
			sdef.density = 1.0f;

			bA->CreateShape(sdef);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(5.0f, 5.0f, 0.0f);

			bB = m_world.CreateBody(bd);

			m_hullB.SetExtents(2.0f, 2.0f, 0.5f);

			b3HullShape hull;
			hull.m_hull = &m_hullB;

			b3ShapeDef sdef;
			sdef.shape = &hull;
			sdef.density = 1.0f;

			bB->CreateShape(sdef);
		}

		// Create prismatic joint
		{
			b3Vec3 anchor(0.0f, 5.0f, 0.0f);
			b3Vec3 axis(1.0f, 0.0f, 0.0f);

			b3PrismaticJointDef jd;
			jd.Initialize(bA, bB, anchor, axis);
			jd.motorSpeed = 10.0f;
			jd.maxMotorForce = 10000.0f;
			jd.enableMotor = true;
			jd.lowerTranslation = 0.0f;
			jd.upperTranslation = 10.0f;
			jd.enableLimit = true;

			m_joint = (b3PrismaticJoint*)m_world.CreateJoint(jd);
		}
	}

	void Step()
	{
		Test::Step();

		g_draw->DrawString(b3Color_white, "L - Enable Limit");
		g_draw->DrawString(b3Color_white, "M - Enable Motor");
		g_draw->DrawString(b3Color_white, "S - Flip Motor Speed");
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_L)
		{
			m_joint->EnableLimit(!m_joint->IsLimitEnabled());
		}
	
		if (button == GLFW_KEY_M)
		{
			m_joint->EnableMotor(!m_joint->IsMotorEnabled());
		}
	
		if (button == GLFW_KEY_S)
		{
			m_joint->SetMotorSpeed(-m_joint->GetMotorSpeed());
		}
	}

	static Test* Create()
	{
		return new PrismaticTest();
	}

	b3BoxHull m_hullA;
	b3BoxHull m_hullB;
	b3PrismaticJoint* m_joint;
};

#endif