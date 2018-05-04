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

		for (u32 i = 0; i < m_count; ++i)
		{
			g_draw->DrawPoint(m_points[i], 4.0f, b3Color_green);
		}

		for (u32 i = 0; i < hull.edgeCount; i += 2)
		{
			const b3HalfEdge* edge = hull.GetEdge(i);
			const b3HalfEdge* twin = hull.GetEdge(i + 1);

			b3Vec3 v1 = hull.GetVertex(edge->origin);
			b3Vec3 v2 = hull.GetVertex(twin->origin);

			g_draw->DrawSegment(v1, v2, b3Color_black);
		}

		g_draw->Flush();

		for (u32 i = 0; i < hull.faceCount; ++i)
		{
			const b3Face* face = hull.GetFace(i);
			const b3HalfEdge* begin = hull.GetEdge(face->edge);

			b3Vec3 n = hull.GetPlane(i).normal;

			const b3HalfEdge* edge = hull.GetEdge(begin->next);
			do
			{
				u32 i1 = begin->origin;
				u32 i2 = edge->origin;
				const b3HalfEdge* next = hull.GetEdge(edge->next);
				u32 i3 = next->origin;

				b3Vec3 v1 = hull.GetVertex(i1);
				b3Vec3 v2 = hull.GetVertex(i2);
				b3Vec3 v3 = hull.GetVertex(i3);

				b3Color solidColor(1.0f, 1.0f, 1.0f, 0.5f);
				g_draw->DrawSolidTriangle(n, v1, v2, v3, solidColor);

				edge = next;
			} while (hull.GetEdge(edge->next) != begin);
		}

		g_draw->Flush();

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