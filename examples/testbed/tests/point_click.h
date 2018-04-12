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

#ifndef POINT_CLICK_H
#define POINT_CLICK_H

class PointClick : public Test
{
public:
	PointClick()
	{
		{
			b3BodyDef bdef;
			b3Body* ground = m_world.CreateBody(bdef);

			b3MeshShape ms;
			ms.m_mesh = &m_groundMesh;

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

	void BeginDragging()
	{
		if (m_bodyDragger.GetBody() == m_character)
		{
			m_bodyDragger.StopDragging();
		}
	}

	void Step()
	{
		if (m_bodyDragger.IsSelected())
		{
			if (m_bodyDragger.GetBody() != m_character)
			{
				b3Vec3 p1 = m_character->GetPosition();
				b3Vec3 p2 = m_bodyDragger.GetPointA();
				
				b3Vec3 n = b3Normalize(p2 - p1);
				const float32 k = 1000.0f;
				b3Vec3 f = k * n;

				m_character->ApplyForceToCenter(f, true);
			}
		}

		Test::Step();
	}

	static Test* Create()
	{
		return new PointClick();
	}

	b3Body* m_character;
};

#endif