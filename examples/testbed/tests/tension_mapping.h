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

#ifndef TENSION_MAPPING_H
#define TENSION_MAPPING_H

// Hot/Cold color map
// See http://paulbourke.net/miscellaneous/colourspace/
static inline b3Color Color(float32 x, float32 a, float32 b)
{
	x = b3Clamp(x, a, b);

	float32 d = b - a;

	b3Color c(1.0f, 1.0f, 1.0f); 

	if (x < a + 0.25f * d) 
	{
		c.r = 0.0f;
		c.g = 4.0f * (x - a) / d;
		return c;
	}
	
	if (x < a + 0.5f * d) 
	{
		c.r = 0.0f;
		c.b = 1.0f + 4.0f * (a + 0.25f * d - x) / d;
		return c;
	}

	if (x < a + 0.75f * d) 
	{
		c.r = 4.0f * (x - a - 0.5f * d) / d;
		c.b = 0.0f;
		return c;
	}

	c.g = 1.0f + 4.0f * (a + 0.75f * d - x) / d;
	c.b = 0.0f;
	return c;
}

class TensionMapping : public SpringClothTest
{
public:
	TensionMapping()
	{
		b3SpringClothDef def;
		def.allocator = &m_clothAllocator;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.ks = 100000.0f;
		def.gravity.Set(0.0f, -10.0f, 0.0f);

		m_cloth.Initialize(def);

		b3AABB3 aabb;
		aabb.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb.m_upper.Set(5.0f, 1.0f, -4.0f);

		for (u32 i = 0; i < def.mesh->vertexCount; ++i)
		{
			if (aabb.Contains(def.mesh->vertices[i]))
			{
				m_cloth.SetType(i, b3MassType::e_staticMass);
			}
		}
	}

	void Step()
	{
		float32 dt = g_testSettings->inv_hertz;

		m_cloth.Step(dt);
		m_cloth.Apply();
		
		b3StackArray<b3Vec3, 100> T;
		m_cloth.GetTension(T);

		for (u32 i = 0; i < m_clothMesh.triangleCount; ++i)
		{
			b3Triangle* t = m_clothMesh.triangles + i;

			const float32 kMaxT = 100000.0f;

			b3Vec3 f1 = T[t->v1];
			float32 L1 = b3Length(f1);
			b3Color c1 = Color(L1, 0.0f, kMaxT);

			b3Vec3 f2 = T[t->v2];
			float32 L2 = b3Length(f2);
			b3Color c2 = Color(L2, 0.0f, kMaxT);

			b3Vec3 f3 = T[t->v3];
			float32 L3 = b3Length(f3);
			b3Color c3 = Color(L3, 0.0f, kMaxT);

			b3Vec3 v1 = m_clothMesh.vertices[t->v1];
			b3Vec3 v2 = m_clothMesh.vertices[t->v2];
			b3Vec3 v3 = m_clothMesh.vertices[t->v3];

			b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
			n1.Normalize();

			b3Vec3 n2 = -n1;

			g_draw->DrawSolidTriangle(n1, v1, c1, v2, c2, v3, c3);
			g_draw->DrawSolidTriangle(n2, v1, c1, v3, c3, v2, c2);
		}

		b3SpringClothStep step = m_cloth.GetStep();

		g_draw->DrawString(b3Color_white, "Iterations = %u", step.iterations);

		if (m_clothDragger.IsSelected() == true)
		{
			g_draw->DrawSegment(m_clothDragger.GetPointA(), m_clothDragger.GetPointB(), b3Color_white);
		}
	}

	static Test* Create()
	{
		return new TensionMapping();
	}

	b3GridMesh<10, 10> m_clothMesh;
};

#endif