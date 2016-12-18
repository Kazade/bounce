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

#ifndef VARYING_RESTITUTION_H
#define VARYING_RESTITUTION_H

class VaryingRestitution : public Test
{
public:
	VaryingRestitution()
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

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(-10.0f, 10.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape ball;
			ball.m_center.SetZero();
			ball.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &ball;
			sd.density = 1.0f;
			sd.restitution = 0.2f;
			
			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(-5.0f, 10.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape ball;
			ball.m_center.SetZero();
			ball.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &ball;
			sd.density = 1.0f;
			sd.restitution = 0.4f;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 10.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape ball;
			ball.m_center.SetZero();
			ball.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &ball;
			sd.density = 1.0f;
			sd.restitution = 0.6f;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(5.0f, 10.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape ball;
			ball.m_center.SetZero();
			ball.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &ball;
			sd.density = 1.0f;
			sd.restitution = 0.8f;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(10.0f, 10.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape ball;
			ball.m_center.SetZero();
			ball.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &ball;
			sd.density = 1.0f;
			sd.restitution = 1.0f;

			body->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new VaryingRestitution();
	}
};

#endif
