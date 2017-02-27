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

#ifndef CHARACTER_H
#define CHARACTER_H

class Character : public Test
{
public:
	Character()
	{
		{
			b3BodyDef bdef;
			b3Body* ground = m_world.CreateBody(bdef);

			b3MeshShape ms;
			ms.m_mesh = m_meshes + e_gridMesh;

			b3ShapeDef sd;
			sd.shape = &ms;
			
			ground->CreateShape(sd);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_dynamicBody;
			bdef.fixedRotationY = true;
			bdef.position.Set(0.0f, 5.0f, 0.0f);
			
			m_character = m_world.CreateBody(bdef);

			b3CapsuleShape cap;
			cap.m_centers[0].Set(0.0f, 1.0f, 0.0f);
			cap.m_centers[1].Set(0.0f, -1.0f, 0.0f);
			cap.m_radius = 1.0f;

			b3ShapeDef sdef;
			sdef.shape = &cap;
			sdef.density = 1.0f;
			sdef.friction = 0.5f;

			m_character->CreateShape(sdef);
		}
	}

	void RayHit()
	{
		if (m_rayHit.shape)
		{
			if (m_rayHit.shape->GetBody() != m_character)
			{
				Test::RayHit();
			}
		}
	}

	void Step()
	{
		if (m_rayHit.shape)
		{
			if (m_rayHit.shape->GetBody() != m_character)
			{
				b3Vec3 point = m_rayHit.point;
				b3Vec3 normal = m_rayHit.normal;

				const b3Transform& xf = m_character->GetTransform();
				b3Vec3 n = point - xf.position;
				n.Normalize();
				
				m_character->ApplyForceToCenter(100.0f * n, true);
				g_debugDraw->DrawSolidCircle(normal, point + (0.05f * normal), 5.0f, b3Color(0.5f, 0.5f, 1.0f, 0.5f));
			}
		}

		Test::Step();
	}

	static Test* Create()
	{
		return new Character();
	}

	b3Body* m_character;
};

#endif
