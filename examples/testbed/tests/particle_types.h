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

#ifndef PARTICLE_TYPES_H
#define PARTICLE_TYPES_H

class ParticleTypes : public PinnedCloth
{
public:
	ParticleTypes()
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

	void SetClothType(b3ParticleType type)
	{
		for (u32 i = 0; i < m_cloth.GetParticleCount(); ++i)
		{
			b3Particle* p = m_cloth.GetParticle(i);
			m_cloth.SetType(p, type);
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetClothType(e_staticParticle);
		}

		if (button == GLFW_KEY_K)
		{
			SetClothType(e_kinematicParticle);
		}

		if (button == GLFW_KEY_D)
		{
			SetClothType(e_dynamicParticle);
		}

		for (u32 i = 0; i < m_cloth.GetParticleCount(); ++i)
		{
			b3Particle* p = m_cloth.GetParticle(i);

			b3Vec3 d;
			d.SetZero();

			if (button == GLFW_KEY_LEFT)
			{
				d.x = -1.0f;
			}

			if (button == GLFW_KEY_RIGHT)
			{
				d.x = 1.0f;
			}

			if (button == GLFW_KEY_UP)
			{
				d.y = 1.0f;
			}

			if (button == GLFW_KEY_DOWN)
			{
				d.y = -1.0f;
			}

			if (button == GLFW_KEY_LEFT ||
				button == GLFW_KEY_RIGHT ||
				button == GLFW_KEY_UP ||
				button == GLFW_KEY_DOWN)
			{
				if (p->type == e_staticParticle)
				{
					m_cloth.Translate(p, d);
				}

				if (p->type == e_kinematicParticle)
				{
					b3Vec3 v = p->velocity;

					v += 5.0f * d;

					m_cloth.SetVelocity(p, d);
				}

				if (p->type == e_dynamicParticle)
				{
					b3Vec3 f = 100.0f * d;

					m_cloth.ApplyForce(p, f);
				}
			}
		}
	}

	static Test* Create()
	{
		return new ParticleTypes();
	}
};

#endif