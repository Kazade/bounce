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

#ifndef LINEAR_TIME_OF_IMPACT_H
#define LINEAR_TIME_OF_IMPACT_H

class LinearTimeOfImpact : public Test
{
public:
	LinearTimeOfImpact()
	{
		m_shapeA.m_hull = &b3BoxHull_identity;
		m_shapeA.m_radius = 0.0f;

		m_shapeB.m_hull = &b3BoxHull_identity;
		m_shapeB.m_radius = 0.0f;

		m_xfA.translation.Set(0.0f, 0.0f, 0.0f);
		m_xfA.rotation.SetIdentity();

		m_xfB.translation.Set(5.0f, 1.0f, 0.0f);
		m_xfB.rotation.SetIdentity();
		
		m_proxyA.Set(&m_shapeA, 0);
		m_proxyB.Set(&m_shapeB, 0);
	}

	void Step()
	{
		b3Vec3 dA = b3Vec3_zero;
		b3Vec3 dB(-10.0f, 0.0f, 0.0f);
		
		b3TOIOutput out = b3TimeOfImpact(m_xfA, m_proxyA, dA, m_xfB, m_proxyB, dB);

		b3TOIOutput::State state = out.state;
		scalar t = out.t;
		u32 iterations = out.iterations;

		if (state == b3TOIOutput::e_touching)
		{
			b3Transform xfA;
			xfA.rotation = m_xfA.rotation;
			xfA.translation = m_xfA.translation + t * dA;

			b3Transform xfB;
			xfB.rotation = m_xfB.rotation;
			xfB.translation = m_xfB.translation + t * dB;

			b3GJKOutput query = b3GJK(xfA, m_proxyA, xfB, m_proxyB, false);
			
			b3Vec3 p1 = query.point1;
			b3Vec3 p2 = query.point2;
			b3Vec3 n1 = b3Normalize(p2 - p1);
			b3Vec3 p = 0.5f * (p1 + p2);

			g_draw->DrawPoint(p, 4.0f, b3Color_green);
			g_draw->DrawSegment(p1, p1 + n1, b3Color_green);

			m_world.DrawShape(xfA, &m_shapeA, b3Color_black);
			m_world.DrawShape(xfB, &m_shapeB, b3Color_black);
		}
		
		if (state == b3TOIOutput::e_failed)
		{
			g_draw->DrawString(b3Color_white, "State = Failed");
		}
		else if (state == b3TOIOutput::e_overlapped)
		{
			g_draw->DrawString(b3Color_white, "State = Overlapped");
		}
		else if (state == b3TOIOutput::e_separated)
		{
			g_draw->DrawString(b3Color_white, "State = Separated!");
		}
		else if (state == b3TOIOutput::e_touching)
		{
			g_draw->DrawString(b3Color_white, "State = Touching!");
		}

		g_draw->DrawString(b3Color_white, "Iterations = %d", out.iterations);

		g_draw->DrawString(b3Color_white, "Left/Right/Up/Down Arrow - Translate shape");
		g_draw->DrawString(b3Color_white, "X/Y/Z - Rotate shape");

		g_draw->DrawTransform(m_xfA);
		g_draw->DrawTransform(m_xfB);

		m_world.DrawShape(m_xfA, &m_shapeA, b3Color_black);
		m_world.DrawShape(m_xfB, &m_shapeB, b3Color_black);

		m_world.DrawSolidShape(m_xfA, &m_shapeA, b3Color_white);
		m_world.DrawSolidShape(m_xfB, &m_shapeB, b3Color_white);

		g_draw->DrawSegment(m_xfA.translation, m_xfA.translation + dA, b3Color_white);
		g_draw->DrawSegment(m_xfB.translation, m_xfB.translation + dB, b3Color_white);
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_LEFT)
		{
			m_xfB.translation.x -= 0.105f;
		}

		if (key == GLFW_KEY_RIGHT)
		{
			m_xfB.translation.x += 0.105f;
		}

		if (key == GLFW_KEY_UP)
		{
			m_xfB.translation.y += 0.105f;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_xfB.translation.y -= 0.105f;
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
		return new LinearTimeOfImpact();
	}

	b3HullShape m_shapeA;
	b3Transform m_xfA;
	b3ShapeGJKProxy m_proxyA;

	b3HullShape m_shapeB;
	b3Transform m_xfB;
	b3ShapeGJKProxy m_proxyB;
};

#endif