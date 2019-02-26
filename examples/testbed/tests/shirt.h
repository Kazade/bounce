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

#ifndef SHIRT_H
#define SHIRT_H

class Shirt : public Test
{
public:
	Shirt()
	{
		// Generate 2D mesh
		m_shirtGarmentMesh.Set(&m_shirtGarment, 0.1f);

		// Create 3D mesh
		m_shirtClothMesh.Set(&m_shirtGarmentMesh);

		// Perform fitting
		for (u32 i = 0; i < 3; ++i)
		{
			b3ClothMeshMesh* front = m_shirtClothMesh.meshes + i;
			for (u32 j = 0; j < front->vertexCount; ++j)
			{
				u32 v = front->startVertex + j;
				m_shirtClothMesh.vertices[v].z = -1.0f;
			}
		}

		for (u32 i = 3; i < 6; ++i)
		{
			b3ClothMeshMesh* back = m_shirtClothMesh.meshes + i;
			for (u32 j = 0; j < back->vertexCount; ++j)
			{
				u32 v = back->startVertex + j;
				m_shirtClothMesh.vertices[v].z = 1.0f;
			}
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_shirtClothMesh;
		def.density = 0.2f;
		def.structural = 10000.0f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));
		m_cloth->SetWorld(&m_world);

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}
	
	~Shirt()
	{
		delete m_clothDragger;
		delete m_cloth;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Step(g_testSettings->inv_hertz);

		m_cloth->Draw();

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_clothSolverIterations);

		float32 E = m_cloth->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->Drag();
		}
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_clothDragger->IsDragging() == false)
		{
			m_clothDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_clothDragger->IsDragging() == true)
		{
			m_clothDragger->StopDragging();
		}
	}

	static Test* Create()
	{
		return new Shirt();
	}

	b3ShirtGarment m_shirtGarment;
	b3GarmentMesh m_shirtGarmentMesh;
	b3GarmentClothMesh m_shirtClothMesh;

	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger;
};

#endif