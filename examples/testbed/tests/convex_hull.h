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

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

class ConvexHull : public Test
{
public:
	enum
	{
		// Half to avoid generation failure due to many vertices
		e_count = B3_MAX_HULL_VERTICES / 2
	};

	ConvexHull()
	{
		Generate();
	}

	~ConvexHull()
	{

	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_G)
		{
			Generate();
		}
	}

	void Generate()
	{
		m_count = 0;
		for (u32 i = 0; i < e_count; ++i)
		{
			float32 x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 z = 3.0f * RandomFloat(-1.0f, 1.0f);
			
			// Clamp to force coplanarities.
			// This will stress the convex hull creation code.
			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);
			
			m_points[m_count++] = p;
		}
	}

	void Step()
	{
		b3QHull hull;
		hull.Set(m_points, m_count);

		b3HullShape shape;
		shape.m_hull = &hull;

		g_draw->DrawSolidShape(&shape, b3Color_white, b3Transform_identity);
		
		g_draw->DrawString(b3Color_white, "G - Generate a random convex hull");
	}

	static Test* Create()
	{
		return new ConvexHull();
	}

	u32 m_count;
	b3Vec3 m_points[e_count];
};

#endif