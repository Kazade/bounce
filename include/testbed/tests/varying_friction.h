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

#ifndef VARYING_FRICTION_H
#define VARYING_FRICTION_H

class VaryingFriction : public Test
{
public:
	VaryingFriction()
	{
		g_camera.m_zoom = 200.0f;
		g_camera.m_q = b3Quat(b3Vec3(1.0f, 0.0f, 0.0f), -0.1f * B3_PI);
		g_camera.m_q = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), -0.1f * B3_PI) * g_camera.m_q;

		{
			b3BodyDef bdef;	
			b3Body* ground = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			ground->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-20.0f, 20.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), -0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 0.4f;
			ramp->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(20.0f, 30.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 0.3f;
			ramp->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-20.0f, 40.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), -0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 0.2f;
			ramp->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(20.0f, 50.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 0.1f;
			ramp->CreateShape(sdef);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, -10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 0.2f;
			sdef.shape = &hs;

			body->CreateShape(sdef);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 0.5f;
			sdef.shape = &hs;

			body->CreateShape(sdef);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 0.8f;
			sdef.shape = &hs;

			body->CreateShape(sdef);
		}
	}

	static Test* Create()
	{
		return new VaryingFriction();
	}
};

#endif
