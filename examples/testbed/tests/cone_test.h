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

#ifndef CONE_TEST_H
#define CONE_TEST_H

class ConeTest : public Test
{
public:
	ConeTest()
	{
		b3Body* ref;
		b3Body* head;
		
		{
			b3BodyDef bd;
			//bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 0.0f, 0.0f);			
			ref = m_world.CreateBody(bd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 2.0f, 0.0f);
			bd.angularVelocity.Set(0.0f, 0.05f * B3_PI, 0.0f);
			head = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 0.15f, 0.0f);
			cs.m_centers[1].Set(0.0f, -0.15f, 0.0f);
			cs.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 10.0f;
			head->CreateShape(sd);
		}

		{
			b3Vec3 anchor(0.0f, 0.0f, 0.0f);
			b3Vec3 axis(0.0f, 1.0f, 0.0f);
			float32 coneAngle = 0.5f * B3_PI;

			b3ConeJointDef cd;
			cd.Initialize(ref, head, axis, anchor, coneAngle);
			cd.enableLimit = true;
			
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}
		
		// Invalidate the orientation
		b3Vec3 axis(1.0f, 0.0f, 0.0f);
		float32 angle = B3_PI;
		head->SetTransform(head->GetPosition(), axis, angle);
	}

	static Test* Create()
	{
		return new ConeTest();
	}
};

#endif
