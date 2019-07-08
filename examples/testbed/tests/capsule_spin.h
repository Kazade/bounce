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

#ifndef CAPSULE_SPIN_H
#define CAPSULE_SPIN_H

class CapsuleSpin : public Test
{
public:
	CapsuleSpin()
	{
		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(0.0f, 10.0f, 0.0f);
			bdef.orientation.Set(b3Vec3(0.0f, 0.0f, -1.0f), 1.5f * B3_PI);
			bdef.linearVelocity.Set(0.005f, -10.0f, 0.005f);
			bdef.angularVelocity.Set(2000.0f * B3_PI, 2000.0f * B3_PI, 10000.0f * B3_PI);
			
			b3Body* body = m_world.CreateBody(bdef);
	
			b3CapsuleShape capsule;
			capsule.m_centers[0].Set(0.0f, 4.0f, 0.0f);
			capsule.m_centers[1].Set(0.0f, -4.0f, 0.0f);
			capsule.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &capsule;
			sd.density = 0.1f;

			body->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new CapsuleSpin();
	}
};

#endif