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
		{
			b3BodyDef bdef;	
			b3Body* ground = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			ground->CreateShape(sd);
		}

		static b3BoxHull rampHull;

		{
			b3Transform xf;
			xf.position.SetZero();
			xf.rotation = b3Diagonal(25.0f, 0.5f, 25.0f);
			rampHull.SetTransform(xf);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-20.0f, 20.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), -0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);
			
			b3HullShape hs;
			hs.m_hull = &rampHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.friction = 0.4f;
			ramp->CreateShape(sd);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(20.0f, 30.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &rampHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.friction = 0.3f;
			ramp->CreateShape(sd);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-20.0f, 40.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), -0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &rampHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.friction = 0.2f;
			ramp->CreateShape(sd);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(20.0f, 50.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &rampHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.friction = 0.1f;
			ramp->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, -10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sd;
			sd.density = 1.0f;
			sd.friction = 0.2f;
			sd.shape = &hs;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sd;
			sd.density = 1.0f;
			sd.friction = 0.5f;
			sd.shape = &hs;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sd;
			sd.density = 1.0f;
			sd.friction = 0.8f;
			sd.shape = &hs;

			body->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new VaryingFriction();
	}
};

#endif