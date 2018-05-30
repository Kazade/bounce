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
	TensionMapping() : m_rectangleGarment(5.0f, 5.0f)
	{
		// Generate 2D mesh
		m_rectangleGarmentMesh.Set(&m_rectangleGarment, 1.0f);

		// Create 3D mesh
		m_rectangleClothMesh.Set(&m_rectangleGarmentMesh);
		
		//  
		b3Mat33 dq = b3Mat33RotationX(0.5f * B3_PI);
		for (u32 i = 0; i < m_rectangleClothMesh.vertexCount; ++i)
		{
			m_rectangleClothMesh.vertices[i] = dq * m_rectangleClothMesh.vertices[i];
		}

		b3ClothDef def;
		def.mesh = &m_rectangleClothMesh;
		def.radius = 0.2f;
		def.density = 0.2f;
		def.structural = 10000.0f;

		m_cloth = m_world.CreateCloth(def);

		b3AABB3 aabb;
		aabb.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb.m_upper.Set(5.0f, 1.0f, -4.0f);

		for (b3Particle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
			if (aabb.Contains(p->GetPosition()))
			{
				p->SetType(e_staticParticle);
			}
		}
	}

	void Step()
	{
		Test::Step();

		m_cloth->Apply();

		b3ClothMesh* mesh = m_cloth->GetMesh();

		b3StackArray<b3Vec3, 256> tension;
		tension.Resize(mesh->vertexCount);
		for (u32 i = 0; i < mesh->vertexCount; ++i)
		{
			tension[i].SetZero();
		}

		for (b3Force* f = m_cloth->GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() == e_springForce)
			{
				b3SpringForce* s = (b3SpringForce*)f;

				u32 v1 = s->GetParticle1()->GetVertex();
				u32 v2 = s->GetParticle2()->GetVertex();

				tension[v1] += s->GetActionForce();
				tension[v2] -= s->GetActionForce();
			}
		}

		for (u32 i = 0; i < m_rectangleClothMesh.triangleCount; ++i)
		{
			b3ClothMeshTriangle* t = m_rectangleClothMesh.triangles + i;

			b3Vec3 v1 = mesh->vertices[t->v1];
			b3Vec3 v2 = mesh->vertices[t->v2];
			b3Vec3 v3 = mesh->vertices[t->v3];

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

		float32 E = m_cloth->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);

		if (m_clothDragger.IsSelected() == true)
		{
			g_draw->DrawSegment(m_clothDragger.GetPointA(), m_clothDragger.GetPointB(), b3Color_white);
		}
	}

	static Test* Create()
	{
		return new TensionMapping();
	}

	b3RectangleGarment m_rectangleGarment;
	b3GarmentMesh m_rectangleGarmentMesh;
	b3GarmentClothMesh m_rectangleClothMesh;
};

#endif