/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#ifndef DISTANCE_H
#define DISTANCE_H

class Distance : public Test
{
public:
	Distance()
	{	
		m_xfA.translation.Set(-5.0f, 0.0f, 0.0f);
		m_xfA.rotation.SetIdentity();
		m_shapeA.m_hull = &b3BoxHull_identity;

		m_xfB.translation.Set(5.0f, 0.0f, 0.0f);
		m_xfB.rotation.SetIdentity();
		m_shapeB.m_hull = &b3BoxHull_identity;

		m_proxyA.Set(&m_shapeA, 0);
		m_proxyB.Set(&m_shapeB, 0);
		m_cache.count = 0;
	}

	void Step()
	{
		b3GJKOutput out = b3GJK(m_xfA, m_proxyA, m_xfB, m_proxyB, false, &m_cache);
		if (0 < m_cache.count && m_cache.count < 4)
		{
			b3GJKFeaturePair featurePair = b3GetFeaturePair(m_cache);

			for (u32 i = 0; i < featurePair.count1; ++i)
			{
				u32 index = featurePair.index1[i];
				g_draw->DrawPoint(m_xfA * m_proxyA.GetVertex(index), 4.0f, b3Color(1.0f, 1.0f, 0.0f));
			}

			for (u32 i = 0; i < featurePair.count2; ++i)
			{
				u32 index = featurePair.index2[i];
				g_draw->DrawPoint(m_xfB * m_proxyB.GetVertex(index), 4.0f, b3Color(1.0f, 1.0f, 0.0f));
			}
		}
		
		g_draw->DrawPoint(out.point1, 4.0f, b3Color(0.0f, 1.0f, 0.0f));
		g_draw->DrawPoint(out.point2, 4.0f, b3Color(0.0f, 1.0f, 0.0f));
		g_draw->DrawSegment(out.point1, out.point2, b3Color(1.0f, 1.0f, 1.0f));

		g_draw->DrawTransform(m_xfA);
		g_draw->DrawTransform(m_xfB);
		
		m_world.DrawShape(m_xfA, &m_shapeA, b3Color_black);
		m_world.DrawShape(m_xfB, &m_shapeB, b3Color_black);

		g_draw->DrawString(b3Color_white, "Left/Right/Up/Down Arrow - Translate shape");
		g_draw->DrawString(b3Color_white, "X/Y/Z - Rotate shape");
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_LEFT)
		{
			m_xfB.translation.x -= 0.05f;
		}
		
		if (key == GLFW_KEY_RIGHT)
		{
			m_xfB.translation.x += 0.05f;
		}
		
		if (key == GLFW_KEY_UP)
		{
			m_xfB.translation.y += 0.05f;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_xfB.translation.y -= 0.05f;
		}

		if (key == GLFW_KEY_X)
		{
			b3Quat qx = b3QuatRotationX(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qx;
		}
		
		if (key == GLFW_KEY_Y)
		{
			b3Quat qy = b3QuatRotationY(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qy;
		}

		if (key == GLFW_KEY_Z)
		{
			b3Quat qz = b3QuatRotationZ(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qz;
		}
	}

	static Test* Create()
	{
		return new Distance();
	}

	b3HullShape m_shapeA;
	b3Transform m_xfA;
	b3ShapeGJKProxy m_proxyA;

	b3HullShape m_shapeB;
	b3Transform m_xfB;
	b3ShapeGJKProxy m_proxyB;
	
	b3SimplexCache m_cache;
};

#endif