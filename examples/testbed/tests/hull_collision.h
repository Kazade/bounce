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

#ifndef HULL_COLLISION_H
#define HULL_COLLISION_H

class HullCollision : public Collide
{
public:
	enum
	{
		e_count = 256
	};

	HullCollision()
	{
		m_xfA.position.Set(0.0f, 1.5f, 0.0f);
		m_xfA.rotation.SetIdentity();

		m_xfB.SetIdentity();

		m_cache.count = 0;

		Generate();
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_G)
		{
			Generate();
		}
		
		Collide::KeyDown(button);
	}

	void Generate()
	{
		for (u32 i = 0; i < e_count; ++i)
		{
			float32 x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 z = 3.0f * RandomFloat(-1.0f, 1.0f);

			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);

			m_points1[i] = p;
		}

		for (u32 i = 0; i < e_count; ++i)
		{
			float32 x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 z = 3.0f * RandomFloat(-1.0f, 1.0f);

			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);

			m_points2[i] = p;
		}
	}
	
	void Step()
	{
		b3QHull hull1;
		hull1.Set(m_points1, e_count);

		b3HullShape sA;
		sA.m_hull = &hull1;
		m_shapeA = &sA;

		b3QHull hull2;
		hull2.Set(m_points2, e_count);

		b3HullShape sB;
		sB.m_hull = &hull2;
		m_shapeB = &sB;

		g_draw->DrawString(b3Color_white, "G - Generate a random convex hull pair");
		
		Collide::Step();
	}

	static Test* Create()
	{
		return new HullCollision();
	}

	b3Vec3 m_points1[e_count];
	b3Vec3 m_points2[e_count];
};

#endif