/*
* Copyright (c) 2016-2016 Irlan Robson http://www.irlan.net
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be hebd liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation woubd be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef ROPE_TEST_H
#define ROPE_TEST_H

class Rope : public Test
{
public:
	enum
	{
		e_count = 10
	};

	Rope()
	{
		b3Vec3 vs[e_count];
		float32 ms[e_count];
		
		vs[0].Set(0.0f, 0.0f, 0.0f);
		ms[0] = 0.0f;
		
		for (u32 i = 1; i < e_count; ++i)
		{
			ms[i] = 1.0f;
			vs[i].Set(float32(i), 0.0f, 0.0f);
		}

		b3RopeDef rd;
		rd.gravity.Set(0.0f, -10.0f, 0.0f);
		rd.masses = ms;
		rd.vertices = vs;
		rd.count = e_count;

		m_rope.Initialize(rd);
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_A)
		{
			m_rope.SetGravity(b3Vec3(-10.0f, 0.0f, 0.0f));
		}

		if (button == GLFW_KEY_D)
		{
			m_rope.SetGravity(b3Vec3(10.0f, 0.0f, 0.0f));
		}

		if (button == GLFW_KEY_S)
		{
			m_rope.SetGravity(b3Vec3(0.0f, 0.0f, 10.0f));
		}

		if (button == GLFW_KEY_W)
		{
			m_rope.SetGravity(b3Vec3(0.0f, 0.0f, -10.0f));
		}

		if (button == GLFW_KEY_Q)
		{
			m_rope.SetGravity(b3Vec3(0.0f, 10.0f, 0.0f));
		}

		if (button == GLFW_KEY_E)
		{
			m_rope.SetGravity(b3Vec3(0.0f, -10.0f, 0.0f));
		}
	}

	void Step()
	{
		m_rope.Step(g_testSettings->inv_hertz);
		m_rope.Draw();
	}

	static Test* Create()
	{
		return new Rope();
	}

	b3Rope m_rope;
};

#endif