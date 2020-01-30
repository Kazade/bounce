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

#ifndef AABB_TIME_OF_IMPACT_H
#define AABB_TIME_OF_IMPACT_H

class AABBTimeOfImpact : public Test
{
public:
	AABBTimeOfImpact()
	{
		b3Vec3 cA = b3Vec3_zero;
		b3Vec3 eA(1.0f, 1.0f, 1.0f);
		m_aabbA.Set(cA, eA);

		b3Vec3 cB(-2.0f, -2.0f, 0.0f);
		b3Vec3 eB(1.0f, 1.0f, 1.0f);
		m_aabbB.Set(cB, eB);

		m_dB.Set(1.0f, 1.0f, 0.0f);

		m_time = 0.0f;
	}

	void Step()
	{
		g_draw->DrawString(b3Color_white, "Arrows - Translate AABB");

		g_draw->DrawAABB(m_aabbA, b3Color_white);
		g_draw->DrawAABB(m_aabbB, b3Color_white);

		b3Vec3 cA = m_aabbA.GetCenter();
		b3Vec3 cB = m_aabbB.GetCenter();

		b3Vec3 eA = m_aabbA.GetExtents();
		b3Vec3 eB = m_aabbB.GetExtents();

		b3Vec3 dA = b3Vec3_zero;
		b3Vec3 dB = m_dB;

		g_draw->DrawSegment(cA, cA + dA, b3Color_white);
		g_draw->DrawSegment(cB, cB + dB, b3Color_white);

		{
			b3Vec3 cBt = cB + m_time * dB;

			b3AABB B;
			B.Set(cBt, eB);

			g_draw->DrawAABB(B, b3Color_red);
		}

		b3TOIOutput out = b3TimeOfImpact(m_aabbA, dA, m_aabbB, dB);

		b3TOIOutput::State state = out.state;
		scalar t = out.t;

		if (state == b3TOIOutput::e_touching)
		{
			b3Vec3 cAt = cA + t * dA;
			b3Vec3 cBt = cB + t * dB;

			b3AABB A;
			A.Set(cAt, eA);

			b3AABB B;
			B.Set(cBt, eB);

			g_draw->DrawAABB(A, b3Color_black);
			g_draw->DrawAABB(B, b3Color_black);
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
	}

	void KeyDown(int key)
	{
		const scalar dt = 0.01f;
		const scalar d = 0.1f;

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
			m_aabbB.lowerBound.x -= d;
			m_aabbB.upperBound.x -= d;
		}

		if (key == GLFW_KEY_RIGHT)
		{
			m_aabbB.lowerBound.x += d;
			m_aabbB.upperBound.x += d;
		}

		if (key == GLFW_KEY_UP)
		{
			m_aabbB.lowerBound.y += d;
			m_aabbB.upperBound.y += d;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_aabbB.lowerBound.y -= d;
			m_aabbB.upperBound.y -= d;
		}
		
		if (key == GLFW_KEY_W)
		{
			m_aabbB.lowerBound.z += d;
			m_aabbB.upperBound.z += d;
		}

		if (key == GLFW_KEY_S)
		{
			m_aabbB.lowerBound.z -= d;
			m_aabbB.upperBound.z -= d;
		}
	}

	static Test* Create()
	{
		return new AABBTimeOfImpact();
	}

	scalar m_time;

	b3AABB m_aabbA;
	b3AABB m_aabbB;
	b3Vec3 m_dB;
};

#endif