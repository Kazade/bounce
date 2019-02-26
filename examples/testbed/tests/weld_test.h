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

#ifndef WELD_TEST_H
#define WELD_TEST_H

class WeldTest : public Test
{
public:
	WeldTest()
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

		b3Body* bA, *bB;

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(-2.0f, 5.05f, 0.0f);
			bA = m_world.CreateBody(bd);

			b3CapsuleShape shape;
			shape.m_centers[0].Set(0.0f, -3.5f, 0.0f);
			shape.m_centers[1].Set(0.0f, 3.5f, 0.0f);
			shape.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &shape;
			sd.density = 1.0f;

			bA->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(1.0f, 5.05f, 0.0f);

			bB = m_world.CreateBody(bd);

			static b3BoxHull doorHull;
			{
				b3Transform xf;
				xf.position.SetZero();
				xf.rotation = b3Diagonal(2.0f, 4.0f, 0.5f);
				doorHull.SetTransform(xf);
			}

			b3HullShape hull;
			hull.m_hull = &doorHull;

			b3ShapeDef sdef;
			sdef.shape = &hull;
			sdef.density = 1.0f;

			bB->CreateShape(sdef);
			
			{
				b3Vec3 anchor(-2.0f, 5.0f, 0.0f);

				b3WeldJointDef jd;
				jd.Initialize(bA, bB, anchor);

				b3WeldJoint* wj = (b3WeldJoint*)m_world.CreateJoint(jd);
			}

			// Invalidate the orientation
			b3Vec3 axis(1.0f, 0.0f, 0.0f);
			float32 angle = B3_PI;
			bB->SetTransform(bB->GetPosition(), axis, angle);
		}
	}

	static Test* Create()
	{
		return new WeldTest();
	}
};

#endif