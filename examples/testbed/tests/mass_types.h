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

#ifndef MASS_TYPES_H
#define MASS_TYPES_H

class MassTypes : public PinnedCloth
{
public:
	MassTypes()
	{
	}

	void Step()
	{
		PinnedCloth::Step();

		g_draw->DrawString(b3Color_white, "S - Static");
		g_draw->DrawString(b3Color_white, "D - Dynamic");
		g_draw->DrawString(b3Color_white, "K - Kinematic");
		g_draw->DrawString(b3Color_white, "Arrows - Apply Force/Velocity/Position");
	}

	void SetClothType(b3MassType type)
	{
		for (u32 i = 0; i < m_cloth.GetMassCount(); ++i)
		{
			m_cloth.SetType(i, type);
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetClothType(b3MassType::e_staticMass);
		}

		if (button == GLFW_KEY_K)
		{
			SetClothType(b3MassType::e_kinematicMass);
		}

		if (button == GLFW_KEY_D)
		{
			SetClothType(b3MassType::e_dynamicMass);
		}

		for (u32 i = 0; i < m_cloth.GetMassCount(); ++i)
		{
			if (m_cloth.GetType(i) == b3MassType::e_staticMass)
			{
				if (button == GLFW_KEY_LEFT)
				{
					b3Vec3 p = m_cloth.GetPosition(i);

					p.x -= 1.0f;

					m_cloth.SetPosition(i, p);
				}

				if (button == GLFW_KEY_RIGHT)
				{
					b3Vec3 p = m_cloth.GetPosition(i);

					p.x += 1.0f;

					m_cloth.SetPosition(i, p);
				}

				if (button == GLFW_KEY_UP)
				{
					b3Vec3 p = m_cloth.GetPosition(i);

					p.z += 1.0f;

					m_cloth.SetPosition(i, p);
				}

				if (button == GLFW_KEY_DOWN)
				{
					b3Vec3 p = m_cloth.GetPosition(i);

					p.z -= 1.0f;

					m_cloth.SetPosition(i, p);
				}
			}

			if (m_cloth.GetType(i) == b3MassType::e_kinematicMass)
			{
				if (button == GLFW_KEY_LEFT)
				{
					b3Vec3 v = m_cloth.GetVelocity(i);

					v.x -= 5.0f;

					m_cloth.SetVelocity(i, v);
				}

				if (button == GLFW_KEY_RIGHT)
				{
					b3Vec3 v = m_cloth.GetVelocity(i);

					v.x += 5.0f;

					m_cloth.SetVelocity(i, v);
				}

				if (button == GLFW_KEY_UP)
				{
					b3Vec3 v = m_cloth.GetVelocity(i);

					v.z -= 5.0f;

					m_cloth.SetVelocity(i, v);
				}

				if (button == GLFW_KEY_DOWN)
				{
					b3Vec3 v = m_cloth.GetVelocity(i);

					v.z += 5.0f;

					m_cloth.SetVelocity(i, v);
				}
			}

			if (m_cloth.GetType(i) == b3MassType::e_dynamicMass)
			{
				if (button == GLFW_KEY_LEFT)
				{
					m_cloth.ApplyForce(i, b3Vec3(-100.0f, 0.0f, 0.0f));
				}

				if (button == GLFW_KEY_RIGHT)
				{
					m_cloth.ApplyForce(i, b3Vec3(100.0f, 0.0f, 0.0f));
				}

				if (button == GLFW_KEY_UP)
				{
					m_cloth.ApplyForce(i, b3Vec3(0.0f, 0.0f, -100.0f));
				}

				if (button == GLFW_KEY_DOWN)
				{
					m_cloth.ApplyForce(i, b3Vec3(0.0f, 0.0f, 100.0f));
				}
			}
		}
	}

	static Test* Create()
	{
		return new MassTypes();
	}
};

#endif