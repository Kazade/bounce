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

#ifndef PINNED_CLOTH_H
#define PINNED_CLOTH_H

class PinnedCloth : public ClothTest
{
public:
	PinnedCloth()
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
		def.r = 0.05f;

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

	static Test* Create()
	{
		return new PinnedCloth();
	}
	
	b3GridMesh<10, 10> m_gridMesh;
	b3ClothMeshMesh m_gridClothMeshMesh;
	b3ClothMesh m_gridClothMesh;
};

#endif