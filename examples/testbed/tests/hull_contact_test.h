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

#ifndef HULL_CONTACT_TEST_H
#define HULL_CONTACT_TEST_H

class HullContactTest : public Test
{
public:
	HullContactTest()
	{
		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 1.0f;

			body->CreateShape(sdef);
		}

		static b3QHull hulls[2];
		
		for (u32 i = 0; i < 2; ++i)
		{
			b3QHull* hull = hulls + i;

			const u32 count = 32;
			b3Vec3 points[count];

			for (u32 j = 0; j < count; ++j)
			{
				// Clamp to force coplanarities.
				// This will stress the generation code.
				float x = 3.0f * RandomFloat(-1.0f, 1.0f);
				float y = 3.0f * RandomFloat(-1.0f, 1.0f);
				float z = 3.0f * RandomFloat(-1.0f, 1.0f);

				x = b3Clamp(x, -1.5f, 1.5f);
				y = b3Clamp(y, -1.5f, 1.5f);
				z = b3Clamp(z, -1.5f, 1.5f);

				b3Vec3 p(x, y, z);

				points[j] = p;
			}

			hull->Set(sizeof(b3Vec3), points, count);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_dynamicBody;
			bdef.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = hulls + 0;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
			sdef.friction = 0.1f;
			sdef.shape = &hs;

			body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_dynamicBody;
			bdef.position.Set(0.0f, 10.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = hulls + 1;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
			sdef.friction = 0.1f;
			sdef.shape = &hs;

			body->CreateShape(sdef);
		}
	}

	static Test* Create()
	{
		return new HullContactTest();
	}
};

#endif