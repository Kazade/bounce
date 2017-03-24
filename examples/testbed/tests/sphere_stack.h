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

#ifndef SPHERE_STACK_H
#define SPHERE_STACK_H

class SphereStack : public Test
{
public:
	enum
	{
		e_rowCount = 1,
		e_columnCount = 5,
		e_depthCount = 1
	};

	SphereStack()
	{
		{
			b3BodyDef bd;
			bd.type = e_staticBody;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;
			
			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.0f;
			sd.friction = 1.0f;
			sd.restitution = 0.0f;
			
			b3Shape* groundShape = ground->CreateShape(sd);
		}

		b3Vec3 stackOrigin;
		stackOrigin.Set(0.0f, 5.0f, 0.0f);
		float32 radius = 1.0f;
		float32 diameter = 2.0f * radius;

		for (u32 i = 0; i < e_rowCount; ++i)
		{
			for (u32 j = 0; j < e_columnCount; ++j)
			{
				for (u32 k = 0; k < e_depthCount; ++k)
				{
					b3BodyDef bdef;
					bdef.type = b3BodyType::e_dynamicBody;
					bdef.position.x = float32(i) * diameter;
					bdef.position.y = float32(j) * diameter;
					bdef.position.z = float32(k) * diameter;
					bdef.position += stackOrigin;
					bdef.linearVelocity.Set(0.0f, -50.0f, 0.0f);

					b3Body* body = m_world.CreateBody(bdef);

					b3SphereShape sphere;
					sphere.m_center.SetZero();
					sphere.m_radius = radius;

					b3ShapeDef sdef;
					sdef.shape = &sphere;
					sdef.density = 1.0f;
					sdef.friction = 0.3f;

					b3Shape* shape = body->CreateShape(sdef);
				}
			}
		}
	}

	static Test* Create()
	{
		return new SphereStack();
	}
};

#endif
