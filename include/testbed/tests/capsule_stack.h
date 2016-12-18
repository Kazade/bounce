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

#ifndef CAPSULE_STACK_H
#define CAPSULE_STACK_H

class CapsuleStack : public Test
{
public:
	CapsuleStack()
	{		
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_staticBody;
			
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.0f;
			sd.friction = 1.0f;
			sd.restitution = 0.0f;
			
			b3Shape* shape = body->CreateShape(sd);
		}
				
		for (float32 y = 2.5f; y < 20.0f; y += 2.5f)
		{
			b3BodyDef bdef;
			bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
			bdef.position.Set(0.0f, y, 0.0f);
			bdef.linearVelocity.Set(0.0f, -1.0f, 0.0f);
			bdef.type = b3BodyType::e_dynamicBody;

			b3Body* body = m_world.CreateBody(bdef);

			b3CapsuleShape capsule;
			capsule.m_centers[0].Set(0.0f, -1.0f, 0.0f);
			capsule.m_centers[1].Set(0.0f, 1.0f, 0.0f);
			capsule.m_radius = 1.0f;

			b3ShapeDef sdef;
			sdef.shape = &capsule;
			sdef.density = 1.0f;
			sdef.friction = 0.3f;

			b3Shape* shape = body->CreateShape(sdef);
		}
	}

	static Test* Create()
	{
		return new CapsuleStack();
	}
};

#endif
