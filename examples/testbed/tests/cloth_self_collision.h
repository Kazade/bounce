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

#ifndef CLOTH_SELF_COLLISION_H
#define CLOTH_SELF_COLLISION_H

class ClothSelfCollision : public Test
{
public:
	enum
	{
		e_w1 = 5,
		e_h1 = 5,
		e_w2 = 5,
		e_h2 = 5
	};

	ClothSelfCollision()
	{
		b3GridClothMesh<e_w1, e_h1> mesh1;
		b3Quat qX = b3QuatRotationX(0.5f * B3_PI);
		for (u32 i = 0; i < mesh1.vertexCount; ++i)
		{
			mesh1.vertices[i] = b3Mul(qX, mesh1.vertices[i]);
			mesh1.vertices[i].y += 5.0f;
		}

		b3GridClothMesh<e_w2, e_h2> mesh2;
		b3Quat qY = b3QuatRotationY(0.5f * B3_PI);
		for (u32 i = 0; i < mesh2.vertexCount; ++i)
		{
			mesh2.vertices[i] = b3Mul(qY * qX, mesh2.vertices[i]);
			mesh2.vertices[i].y += 12.0f;
		}

		// Merge the meshes
		m_clothMesh.vertexCount = mesh1.vertexCount + mesh2.vertexCount;
		m_clothMesh.vertices = (b3Vec3*)b3Alloc(m_clothMesh.vertexCount * sizeof(b3Vec3));
		
		u32* newVertices1 = (u32*)b3Alloc(mesh1.vertexCount * sizeof(u32));
		u32 vertexIndex = 0;
		for (u32 i = 0; i < mesh1.vertexCount; ++i)
		{
			newVertices1[i] = vertexIndex;
			m_clothMesh.vertices[vertexIndex++] = mesh1.vertices[i];
		}

		u32* newVertices2 = (u32*)b3Alloc(mesh2.vertexCount * sizeof(u32));
		for (u32 i = 0; i < mesh2.vertexCount; ++i)
		{
			newVertices2[i] = vertexIndex;
			m_clothMesh.vertices[vertexIndex++] = mesh2.vertices[i];
		}
		
		m_clothMesh.triangleCount = mesh1.triangleCount + mesh2.triangleCount;
		m_clothMesh.triangles = (b3ClothMeshTriangle*)b3Alloc(m_clothMesh.triangleCount * sizeof(b3ClothMeshTriangle));
		u32 triangleIndex = 0;
		for (u32 i = 0; i < mesh1.triangleCount; ++i)
		{
			m_clothMesh.triangles[triangleIndex].v1 = newVertices1[mesh1.triangles[i].v1];
			m_clothMesh.triangles[triangleIndex].v2 = newVertices1[mesh1.triangles[i].v2];
			m_clothMesh.triangles[triangleIndex].v3 = newVertices1[mesh1.triangles[i].v3];
			++triangleIndex;
		}

		for (u32 i = 0; i < mesh2.triangleCount; ++i)
		{
			m_clothMesh.triangles[triangleIndex].v1 = newVertices2[mesh2.triangles[i].v1];
			m_clothMesh.triangles[triangleIndex].v2 = newVertices2[mesh2.triangles[i].v2];
			m_clothMesh.triangles[triangleIndex].v3 = newVertices2[mesh2.triangles[i].v3];
			++triangleIndex;
		}

		m_clothMesh.meshCount = 1;
		m_clothMesh.meshes = (b3ClothMeshMesh*)b3Alloc(sizeof(b3ClothMeshMesh));
		m_clothMesh.meshes->startTriangle = 0;
		m_clothMesh.meshes->triangleCount = m_clothMesh.triangleCount;
		m_clothMesh.meshes->startVertex = 0;
		m_clothMesh.meshes->vertexCount = m_clothMesh.vertexCount;

		m_clothMesh.shearingLineCount = 0;
		m_clothMesh.bendingLineCount = 0;
		m_clothMesh.sewingLineCount = 0;

		// Create the cloth
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 1.0f;
		def.streching = 100000.0f;
		def.thickness = 0.2f;
		def.friction = 0.3f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));
		m_cloth->EnableSelfCollision(true);

		for (u32 i = 0; i < mesh1.vertexCount; ++i)
		{
			u32 newVertex = newVertices1[i];

			m_cloth->GetParticle(newVertex)->SetType(e_staticClothParticle);
		}

		b3Free(newVertices1);
		b3Free(newVertices2);

		{
			b3BodyDef bd;

			b3Body* b = m_world.CreateBody(bd);

			b3HullShape hullShape;
			hullShape.m_hull = &m_groundHull;
			hullShape.m_radius = 0.0f;;

			b3ShapeDef sd;
			sd.shape = &hullShape;
			sd.friction = 1.0f;

			b3Shape* s = b->CreateShape(sd);

			b3ClothWorldShapeDef csd;
			csd.shape = s;

			m_cloth->CreateWorldShape(csd);
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~ClothSelfCollision()
	{
		b3Free(m_clothMesh.vertices);
		b3Free(m_clothMesh.triangles);
		b3Free(m_clothMesh.meshes);

		delete m_cloth;
		delete m_clothDragger;
	}

	void Step()
	{
		Test::Step();

		m_cloth->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_cloth->Draw();

		if (m_clothDragger->IsDragging())
		{
			b3Vec3 pA = m_clothDragger->GetPointA();
			b3Vec3 pB = m_clothDragger->GetPointB();

			g_draw->DrawPoint(pA, 4.0f, b3Color_green);

			g_draw->DrawPoint(pB, 4.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		g_draw->DrawString(b3Color_white, "S - Turn on/off self collision");
		if (m_cloth->IsSelfCollisionEnabled())
		{
			g_draw->DrawString(b3Color_white, "Self collision enabled");
		}
		else
		{
			g_draw->DrawString(b3Color_white, "Self collision disabled");
		}

		extern u32 b3_clothSolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_clothSolverIterations);

		scalar E = m_cloth->GetEnergy();
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

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_S)
		{
			m_cloth->EnableSelfCollision(!m_cloth->IsSelfCollisionEnabled());
		}
	}

	static Test* Create()
	{
		return new ClothSelfCollision();
	}

	b3ClothMesh m_clothMesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger; 
};

#endif