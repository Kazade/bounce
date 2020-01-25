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

#ifndef SPHERE_STACK_H
#define SPHERE_STACK_H

class SphereStack : public Test
{
public:
	enum
	{
		e_h = 5,
		e_w = 1,
		e_d = 1
	};

	SphereStack()
	{
		{
			b3BodyDef bdef;
			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 1.0f;

			body->CreateShape(sdef);
		}

		scalar e = 1.0f;

		b3SphereShape sphere;
		sphere.m_center.SetZero();
		sphere.m_radius = e;

		b3Vec3 separation;
		separation.x = 1.0f;
		separation.y = 1.0f;
		separation.z = 1.0f;

		b3Vec3 scale;
		scale.x = 2.0f * e + separation.x;
		scale.y = 2.0f * e + separation.y;
		scale.z = 2.0f * e + separation.z;

		b3Vec3 size;
		size.x = 2.0f * e + scale.x * scalar(e_w - 1);
		size.y = 2.0f * e + scale.y * scalar(e_h - 1);
		size.z = 2.0f * e + scale.z * scalar(e_d - 1);

		b3Vec3 translation;
		translation.x = e - 0.5f * size.x;
		translation.y = e - 0.5f * size.y;
		translation.z = e - 0.5f * size.z;

		translation.y += 9.0f;

		for (u32 i = 0; i < e_h; ++i)
		{
			for (u32 j = 0; j < e_w; ++j)
			{
				for (u32 k = 0; k < e_d; ++k)
				{
					b3BodyDef bdef;
					bdef.type = e_dynamicBody;

					bdef.position.Set(scalar(j), scalar(i), scalar(k));

					bdef.position.x *= scale.x;
					bdef.position.y *= scale.y;
					bdef.position.z *= scale.z;

					bdef.position += translation;

					b3Body* body = m_world.CreateBody(bdef);

					b3ShapeDef sdef;
					sdef.density = 0.1f;
					sdef.friction = 0.3f;
					sdef.shape = &sphere;

					body->CreateShape(sdef);

					u32 bodyIndex = GetBodyIndex(i, j, k);

					m_bodies[bodyIndex] = body;
				}
			}
		}
	}

	u32 GetBodyIndex(u32 i, u32 j, u32 k)
	{
		B3_ASSERT(i < e_h);
		B3_ASSERT(j < e_w);
		B3_ASSERT(k < e_d);
		return k + e_d * (j + e_w * i);
	}

	static Test* Create()
	{
		return new SphereStack();
	}

	b3Body* m_bodies[e_h * e_w * e_d];
};

#endif