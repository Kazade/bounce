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

class TensionMapping : public ClothTest
{
public:
	TensionMapping()
	{
		m_gridClothMesh.vertexCount = m_gridMesh.vertexCount;
		m_gridClothMesh.vertices = m_gridMesh.vertices;

		m_gridClothMesh.triangleCount = m_gridMesh.triangleCount;
		m_gridClothMesh.triangles = (b3ClothMeshTriangle*)m_gridMesh.triangles;

		m_gridClothMeshMesh.vertexCount = m_gridClothMesh.vertexCount;
		m_gridClothMeshMesh.startVertex = 0;
		
		m_gridClothMeshMesh.triangleCount = m_gridClothMesh.triangleCount;
		m_gridClothMeshMesh.startTriangle = 0;

		m_gridClothMesh.meshCount = 1;
		m_gridClothMesh.meshes = &m_gridClothMeshMesh;

		m_gridClothMesh.sewingLineCount = 0;
		m_gridClothMesh.sewingLines = nullptr;
		
		b3ClothDef def;
		def.mesh = &m_gridClothMesh;
		def.density = 0.2f;
		def.ks = 10000.0f;
		def.kd = 0.0f;
		def.r = 0.2f;

		m_cloth = m_world.CreateCloth(def);

		b3AABB3 aabb;
		aabb.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb.m_upper.Set(5.0f, 1.0f, -4.0f);

		for (u32 i = 0; i < m_cloth->GetParticleCount(); ++i)
		{
			b3Particle* p = m_cloth->GetParticle(i);
			if (aabb.Contains(p->position))
			{
				m_cloth->SetType(p, e_staticParticle);
			}
		}
	}

	void Step()
	{
		float32 dt = g_testSettings->inv_hertz;

		m_cloth->Apply();

		b3StackArray<b3Vec3, 256> tension;
		tension.Resize(m_cloth->GetParticleCount());
		for (u32 i = 0; i < tension.Count(); ++i)
		{
			tension[i].SetZero();
		}

		for (u32 i = 0; i < m_cloth->GetSpringCount(); ++i)
		{
			b3Spring* s = m_cloth->GetSpring(i);

			b3Particle* p1 = s->p1;
			b3Particle* p2 = s->p2;

			u32 i1 = m_cloth->GetParticleIndex(p1);
			u32 i2 = m_cloth->GetParticleIndex(p2);

			tension[i1] += s->tension;
			tension[i2] -= s->tension;
		}

		for (u32 i = 0; i < m_gridClothMesh.triangleCount; ++i)
		{
			b3ClothMeshTriangle* t = m_gridClothMesh.triangles + i;

			b3Particle* p1 = m_cloth->GetParticle(t->v1);
			b3Particle* p2 = m_cloth->GetParticle(t->v2);
			b3Particle* p3 = m_cloth->GetParticle(t->v3);

			b3Vec3 v1 = p1->position;
			b3Vec3 v2 = p2->position;
			b3Vec3 v3 = p3->position;

			b3Draw_draw->DrawSegment(v1, v2, b3Color_black);
			b3Draw_draw->DrawSegment(v2, v3, b3Color_black);
			b3Draw_draw->DrawSegment(v3, v1, b3Color_black);

			b3Vec3 f1 = tension[t->v1];
			float32 L1 = b3Length(f1);

			b3Vec3 f2 = tension[t->v2];
			float32 L2 = b3Length(f2);

			b3Vec3 f3 = tension[t->v3];
			float32 L3 = b3Length(f3);

			float32 L = (L1 + L2 + L3) / 3.0f;

			const float32 kMaxT = 100000.0f;
			b3Color color = Color(L, 0.0f, kMaxT);
			
			b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
			n1.Normalize();
			g_draw->DrawSolidTriangle(n1, v1, v2, v3, color);

			b3Vec3 n2 = -n1;
			g_draw->DrawSolidTriangle(n2, v1, v3, v2, color);
		}

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %u", b3_clothSolverIterations);

		if (m_clothDragger.IsSelected() == true)
		{
			g_draw->DrawSegment(m_clothDragger.GetPointA(), m_clothDragger.GetPointB(), b3Color_white);
		}
	}

	static Test* Create()
	{
		return new TensionMapping();
	}

	b3GridMesh<10, 10> m_gridMesh;
	b3ClothMeshMesh m_gridClothMeshMesh;
	b3ClothMesh m_gridClothMesh;
};

#endif