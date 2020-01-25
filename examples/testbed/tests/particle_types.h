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

	void SetClothType(b3ClothParticleType type)
	{
		for (u32 j = 0; j < e_w + 1; ++j)
		{
			u32 v = m_clothMesh.GetVertex(0, j);

			b3ClothParticle* p = m_cloth->GetParticle(v);
			p->SetType(type);
		}

		for (u32 j = 0; j < e_w + 1; ++j)
		{
			u32 v = m_clothMesh.GetVertex(e_h, j);

			b3ClothParticle* p = m_cloth->GetParticle(v);
			p->SetType(type);
		}
	}

	void KeyDown(int button)
	{
		if (button == GLFW_KEY_S)
		{
			SetClothType(e_staticClothParticle);
		}

		if (button == GLFW_KEY_K)
		{
			SetClothType(e_kinematicClothParticle);
		}

		if (button == GLFW_KEY_D)
		{
			SetClothType(e_dynamicClothParticle);
		}

		for (b3ClothParticle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
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
				if (p->GetType() == e_staticClothParticle)
				{
					p->ApplyTranslation(d);
				}

				if (p->GetType() == e_kinematicClothParticle)
				{
					b3Vec3 v = p->GetVelocity();

					v += 5.0f * d;

					p->SetVelocity(v);
				}

				if (p->GetType() == e_dynamicClothParticle)
				{
					b3Vec3 f = 100.0f * d;

					p->ApplyForce(f);
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