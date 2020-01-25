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

#ifndef CONVEYOR_BELT_H
#define CONVEYOR_BELT_H

class ConveyorBelt : public Test
{
public:

	ConveyorBelt()
	{
		{
			// Ground
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			body->CreateShape(sd);
		}

		{
			// Platform
			b3BodyDef bd;
			bd.position.Set(0.0f, 5.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			m_platformHull.SetExtents(10.0f, 0.5f, 2.0f);

			b3HullShape hs;
			hs.m_hull = &m_platformHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.friction = 0.8f;
			m_platform = body->CreateShape(sd);
		}

		// Boxes
		m_boxHull.SetExtents(0.5f, 0.5f, 0.5f);
		for (u32 i = 0; i < 5; ++i)
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(2.0f * i, 7.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.2f;
			
			body->CreateShape(sd);
		}
	}

	void PreSolve(b3Contact* contact)
	{
		Test::PreSolve(contact);

		b3Shape* shapeA = contact->GetShapeA();
		b3Shape* shapeB = contact->GetShapeB();

		if (shapeA == m_platform)
		{
			for (u32 i = 0; i < contact->GetManifoldCount(); ++i)
			{
				b3Manifold* manifold = contact->GetManifold(i);

				manifold->motorSpeed = 0.25f * B3_PI;
				manifold->tangentSpeed2 = -2.0f;
			}
		}

		if (shapeB == m_platform)
		{
			for (u32 i = 0; i < contact->GetManifoldCount(); ++i)
			{
				b3Manifold* manifold = contact->GetManifold(i);

				manifold->motorSpeed = -0.25f * B3_PI;
				manifold->tangentSpeed2 = 2.0f;
			}
		}
	}

	static Test* Create()
	{
		return new ConveyorBelt();
	}

	b3BoxHull m_platformHull;
	b3Shape* m_platform;
	b3BoxHull m_boxHull;
};

#endif