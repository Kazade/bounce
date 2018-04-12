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

#ifndef CLOTH_H
#define CLOTH_H

class Cloth : public Test
{
public:
	Cloth()
	{
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.gravity.Set(0.0f, -10.0f, 0.0f);
		def.k1 = 0.2f;
		def.k2 = 0.1f;
		def.kd = 0.005f;
		def.r = 1.0f;

		m_cloth.Initialize(def);

		b3AABB3 aabb;
		aabb.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb.m_upper.Set(5.0f, 1.0f, -4.0f);

		b3Particle* vs = m_cloth.GetVertices();
		for (u32 i = 0; i < m_cloth.GetVertexCount(); ++i)
		{
			if (aabb.Contains(vs[i].p))
			{
				vs[i].im = 0.0f;
			}
		}
	}

	void Step()
	{
		m_cloth.Step(g_testSettings->inv_hertz, g_testSettings->positionIterations);
		m_cloth.Draw();
	}

	static Test* Create()
	{
		return new Cloth();
	}

	b3GridMesh<10, 10> m_clothMesh;
	b3Cloth m_cloth;
};

#endif