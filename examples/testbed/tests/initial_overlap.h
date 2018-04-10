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

#ifndef INITIAL_OVERLAP_H
#define INITIAL_OVERLAP_H

class InitialOverlap : public Test
{
public:
	InitialOverlap()
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

		b3Vec3 boxScale(1.0f, 0.5f, 2.0f);

		static b3BoxHull boxHull;

		b3Transform m;
		m.rotation = b3Diagonal(boxScale.x, boxScale.y, boxScale.z);
		m.position.SetZero();

		boxHull.SetTransform(m);

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 1.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.1f;
			sd.friction = 0.3f;

			body->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 1.5f, 0.0f);

			b3Quat q_y(b3Vec3(0.0f, 1.0f, 0.0f), 0.4f * B3_PI);
			b3Quat q_z(b3Vec3(0.0f, 0.0f, 1.0f), 0.04f * B3_PI);
			b3Quat q = q_z * q_y;

			bd.orientation = q;

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.1f;
			sd.friction = 0.3f;

			body->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new InitialOverlap();
	}
};

#endif