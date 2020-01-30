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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		m_shapeA.m_hull = &b3BoxHull_identity;
		m_shapeA.m_radius = 0.0f;

		m_shapeB.m_hull = &b3BoxHull_identity;
		m_shapeB.m_radius = 0.0f;

		m_sweepA.localCenter.SetZero();
		m_sweepA.worldCenter0.Set(0.0f, 0.0f, 5.0f);
		m_sweepA.worldCenter.Set(0.0f, 0.0f, -5.0f);
		m_sweepA.orientation0.SetIdentity();
		m_sweepA.orientation.SetIdentity();

		m_sweepB.localCenter.SetZero();
		m_sweepB.worldCenter0.Set(5.0f, 0.0f, 0.0f);
		m_sweepB.worldCenter.Set(-5.0f, 0.0f, 0.0f);
		m_sweepB.orientation0.SetIdentity();
		m_sweepB.orientation.SetIdentity();
		
		m_proxyA.Set(&m_shapeA, 0);
		m_proxyB.Set(&m_shapeB, 0);

		m_time = 0.0f;
	}

	void Step()
	{
		b3Color colorA0(1.0f, 0.0f, 0.0f, 1.0f);
		b3Color colorB0(0.0f, 1.0f, 0.0f, 1.0f);

		// t0
		b3Transform xfA0 = m_sweepA.GetTransform(0.0f);
		b3Transform xfB0 = m_sweepB.GetTransform(0.0f);

		g_draw->DrawTransform(xfA0);
		g_draw->DrawTransform(xfB0);

		m_world.DrawShape(xfA0, &m_shapeA, b3Color_black);
		m_world.DrawShape(xfB0, &m_shapeB, b3Color_black);

		m_world.DrawSolidShape(xfA0, &m_shapeA, colorA0);
		m_world.DrawSolidShape(xfB0, &m_shapeB, colorB0);

		// t1
		b3Transform xfA1 = m_sweepA.GetTransform(1.0f);
		b3Transform xfB1 = m_sweepB.GetTransform(1.0f);

		g_draw->DrawTransform(xfA1);
		g_draw->DrawTransform(xfB1);

		m_world.DrawShape(xfA1, &m_shapeA, b3Color_black);
		m_world.DrawShape(xfB1, &m_shapeB, b3Color_black);

		m_world.DrawSolidShape(xfA1, &m_shapeA, colorA0);
		m_world.DrawSolidShape(xfB1, &m_shapeB, colorB0);

		// time
		b3Color colorAt(1.0f, 0.0f, 0.0f, 0.5f);
		b3Color colorBt(0.0f, 1.0f, 0.0f, 0.5f);

		b3Transform xfAx = m_sweepA.GetTransform(m_time);
		b3Transform xfBx = m_sweepB.GetTransform(m_time);

		g_draw->DrawTransform(xfAx);
		g_draw->DrawTransform(xfBx);

		m_world.DrawShape(xfAx, &m_shapeA, b3Color_black);
		m_world.DrawShape(xfBx, &m_shapeB, b3Color_black);

		m_world.DrawSolidShape(xfAx, &m_shapeA, colorAt);
		m_world.DrawSolidShape(xfBx, &m_shapeB, colorBt);

		b3TOIInput input;
		input.proxyA = m_proxyA;
		input.sweepA = m_sweepA;
		input.proxyB = m_proxyB;
		input.sweepB = m_sweepB;
		input.tMax = 1.0f;

		b3TOIOutput output = b3TimeOfImpact(input);

		if (output.state == b3TOIOutput::e_touching)
		{
			b3Transform xfAt = m_sweepA.GetTransform(output.t);
			b3Transform xfBt = m_sweepB.GetTransform(output.t);

			m_world.DrawShape(xfAt, &m_shapeA, b3Color_black);
			m_world.DrawShape(xfBt, &m_shapeB, b3Color_black);
		}

		g_draw->DrawString(b3Color_white, "Left/Right/Up/Down Arrow/W/S - Translate shape");
		g_draw->DrawString(b3Color_white, "X/Y/Z - Rotate shape");
		g_draw->DrawString(b3Color_white, "F/B - Advance Time Forwards/Backwards");
		g_draw->DrawString(b3Color_white, "Iterations = %d", output.iterations);

		if (output.state == b3TOIOutput::e_failed)
		{
			g_draw->DrawString(b3Color_white, "State = Failed");
		}
		else if (output.state == b3TOIOutput::e_overlapped)
		{
			g_draw->DrawString(b3Color_white, "State = Overlapped");
		}
		else if (output.state == b3TOIOutput::e_separated)
		{
			g_draw->DrawString(b3Color_white, "State = Separated!");
		}
		else if (output.state == b3TOIOutput::e_touching)
		{
			g_draw->DrawString(b3Color_white, "State = Touching!");
		}

		g_draw->DrawString(b3Color_white, m_sweepA.worldCenter0, "t0");
		g_draw->DrawString(b3Color_white, m_sweepA.worldCenter, "t1");

		g_draw->DrawString(b3Color_white, m_sweepB.worldCenter0, "t0");
		g_draw->DrawString(b3Color_white, m_sweepB.worldCenter, "t1");
	}

	void KeyDown(int key)
	{
		const scalar dt = 0.01f;
		const scalar d = 0.15f;
		const scalar theta = 0.05f * B3_PI;

		if (key == GLFW_KEY_F)
		{
			m_time += dt;
			if (m_time > 1.0f)
			{
				m_time = 0.0f;
			}
		}

		if (key == GLFW_KEY_B)
		{
			m_time -= dt;
			if (m_time < 0.0f)
			{
				m_time = 1.0f;
			}
		}
		
		if (key == GLFW_KEY_LEFT)
		{
			m_sweepB.worldCenter0.x -= d;
		}

		if (key == GLFW_KEY_RIGHT)
		{
			m_sweepB.worldCenter0.x += d;
		}

		if (key == GLFW_KEY_UP)
		{
			m_sweepB.worldCenter0.y += d;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_sweepB.worldCenter0.y -= d;
		}

		if (key == GLFW_KEY_S)
		{
			m_sweepB.worldCenter0.z += d;
		}

		if (key == GLFW_KEY_W)
		{
			m_sweepB.worldCenter0.z -= d;
		}
		
		if (key == GLFW_KEY_X)
		{
			b3Quat qx = b3QuatRotationX(theta);

			m_sweepB.orientation0 = m_sweepB.orientation0 * qx;
		}

		if (key == GLFW_KEY_Y)
		{
			b3Quat qy = b3QuatRotationY(theta);

			m_sweepB.orientation0 = m_sweepB.orientation0 * qy;
		}

		if (key == GLFW_KEY_Z)
		{
			b3Quat qz = b3QuatRotationZ(theta);

			m_sweepB.orientation0 = m_sweepB.orientation0 * qz;
		}
	}

	static Test* Create()
	{
		return new TimeOfImpact();
	}

	scalar m_time;

	b3HullShape m_shapeA;
	b3Sweep m_sweepA;
	b3ShapeGJKProxy m_proxyA;

	b3HullShape m_shapeB;
	b3Sweep m_sweepB;
	b3ShapeGJKProxy m_proxyB;
};

#endif