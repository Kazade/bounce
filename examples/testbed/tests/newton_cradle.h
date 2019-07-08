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

#ifndef NEWTON_CRADLE_H
#define NEWTON_CRADLE_H

class NewtonCradle : public Test
{
public:
	NewtonCradle()
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

		b3CapsuleShape edge;
		edge.m_centers[0].Set(0.0f, -10.0f, 0.0f);
		edge.m_centers[1].Set(0.0f, 10.0f, 0.0f);
		edge.m_radius = 0.5f;

		b3Body* frame1, *frame2;

		{
			b3BodyDef bd;
			bd.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
			bd.position.Set(0.0f, 10.0f, -5.0f);

			frame1 = m_world.CreateBody(bd);

			b3ShapeDef sd;
			sd.shape = &edge;

			frame1->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
			bd.position.Set(0.0f, 10.0f, 5.0f);

			frame2 = m_world.CreateBody(bd);

			b3ShapeDef sd;
			sd.shape = &edge;

			frame2->CreateShape(sd);
		}

		b3Vec3 center;
		center.Set(-5.0f, 4.0f, 0.0f);

		b3SphereShape vertex;
		vertex.m_center.SetZero();
		vertex.m_radius = 1.0f;

		u32 count = 6;
		for (u32 i = 0; i < count; ++i)
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position = center;
			if (i == count - 1)
			{
				bd.linearVelocity.x = 5.0f;
			}

			b3Body* ball = m_world.CreateBody(bd);

			b3ShapeDef sd;
			sd.shape = &vertex;
			sd.density = 1.0f;
			sd.friction = 0.8f;
			
			ball->CreateShape(sd);
			
			b3Vec3 c1;
			c1.x = center.x;
			c1.y = 10.0f;
			c1.z = 0.0f;

			b3SphereJointDef jd1;
			jd1.bodyA = frame1;
			jd1.collideLinked = true;
			jd1.bodyB = ball;
			jd1.localAnchorA = b3MulT(frame1->GetTransform(), c1);
			jd1.localAnchorB = b3MulT(ball->GetTransform(), c1);
			
			m_world.CreateJoint(jd1);

			center.x += 2.0f * vertex.m_radius;
		}
	}

	static Test* Create()
	{
		return new NewtonCradle();
	}
};

#endif
