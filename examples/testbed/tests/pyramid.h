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

#ifndef PYRAMID_H
#define PYRAMID_H

class Pyramid : public Test
{
public:
	enum
	{
		e_count = 10,
	};

	Pyramid()
	{
		g_camera.m_zoom = 100.0f;

		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			ground->CreateShape(sd);
		}

		b3Vec3 boxSize;
		boxSize.Set(2.0f, 2.0f, 2.0f);

		// shift to ground center
		b3Vec3 translation;
		translation.x = -0.5f * float32(e_count) * boxSize.x;
		translation.y = 1.5f * boxSize.y;
		translation.z = -0.5f * float32(e_count) * boxSize.z;

		u32 count = e_count;
		for (u32 i = 0; i < e_count; ++i)
		{
			u32 j0 = 0, k0 = 0;
			for (u32 j = j0; j < count; ++j)
			{
				for (u32 k = k0; k < count; ++k)
				{
					b3BodyDef bd;
					bd.type = b3BodyType::e_dynamicBody;
					bd.position.x = 1.05f * float32(j) * boxSize.x;
					bd.position.y = 0.0f;
					bd.position.z = 1.05f * float32(k) * boxSize.z;
					bd.position += translation;

					b3Body* body = m_world.CreateBody(bd);

					b3HullShape hs;
					hs.m_hull = &m_boxHull;

					b3ShapeDef sd;
					sd.shape = &hs;
					sd.density = 0.5f;
					sd.friction = 0.5f;

					body->CreateShape(sd);
				}
			}

			// reduce dimension
			++j0;
			++k0;
			--count;
			
			// increment column
			translation.y += 1.5f * boxSize.y;
			// track offset
			translation.x += 0.5f * boxSize.x;
			translation.z += 0.5f * boxSize.z;
		}
	}

	static Test* Create()
	{
		return new Pyramid();
	}
};

#endif
