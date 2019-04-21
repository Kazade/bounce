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

#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <bounce/meshgen/sphere_mesh.h>

struct b3QClothMesh : public b3ClothMesh
{
	b3StackArray<b3Vec3, 256> clothVertices;
	b3StackArray<b3ClothMeshTriangle, 256> clothTriangles;
	b3ClothMeshMesh clothMesh;

	b3QClothMesh()
	{
		vertices = clothVertices.Begin();
		vertexCount = 0;
		triangles = clothTriangles.Begin();
		triangleCount = 0;
		meshCount = 1;
		meshes = &clothMesh;
		sewingLineCount = 0;
		sewingLines = nullptr;
	}

	void SetAsSphere(float32 radius = 1.0f)
	{
		smMesh mesh;
		smCreateMesh(mesh, 2);

		clothVertices.Resize(mesh.vertexCount);
		for (u32 i = 0; i < mesh.vertexCount; ++i)
		{
			clothVertices[i] = radius * mesh.vertices[i];
		}

		clothTriangles.Resize(mesh.indexCount / 3);
		for (u32 i = 0; i < mesh.indexCount / 3; ++i)
		{
			triangles[i].v1 = mesh.indices[3 * i + 0];
			triangles[i].v2 = mesh.indices[3 * i + 1];
			triangles[i].v3 = mesh.indices[3 * i + 2];
		}

		clothMesh.startTriangle = 0;
		clothMesh.triangleCount = clothTriangles.Count();
		clothMesh.startVertex = 0;
		clothMesh.vertexCount = clothVertices.Count();

		vertexCount = clothVertices.Count();
		triangleCount = clothTriangles.Count();
	}
};

class SoftBody : public Test
{
public:
	SoftBody()
	{
		m_mesh.SetAsSphere(2.0f);

		// Translate the cloth mesh upwards
		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			m_mesh.vertices[i].y += 10.0f;
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.structural = 10000.0f;
		def.bending = 0.0f;
		def.damping = 0.0f;

		m_cloth = new b3Cloth(def);

		for (b3Particle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
			p->SetRadius(0.05f);
			p->SetFriction(0.5f);
		}

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		
		m_cloth->SetGravity(gravity);

		// Attach a world to the cloth
		m_cloth->SetWorld(&m_world);

		// Create static shapes
		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			static b3BoxHull boxHull(10.0f, 1.0f, 10.0f);

			b3HullShape boxShape;
			boxShape.m_hull = &boxHull;

			b3ShapeDef sd;
			sd.shape = &boxShape;
			sd.friction = 0.5f;

			b->CreateShape(sd);
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~SoftBody()
	{
		delete m_clothDragger;
		delete m_cloth;
	}

	// Compute the volume of soft body
	float32 ComputeVolume() const
	{
		const b3ClothMesh* mesh = m_cloth->GetMesh();
		
		b3StackArray<b3Vec3, 256> positions;
		positions.Resize(mesh->vertexCount);
		
		for (u32 i = 0; i < mesh->vertexCount; ++i)
		{
			b3Particle* p = m_cloth->GetVertexParticle(i);
			positions[i] = p->GetPosition();
		}

		b3AABB3 aabb;
		aabb.Compute(positions.Begin(), positions.Count());

		return aabb.Volume();
	}

	// Apply pressure forces
	// Explanation available in the paper 
	// "Pressure Model of Soft Body Simulation"
	void ApplyPressureForces()
	{
		const b3ClothMesh* mesh = m_cloth->GetMesh();

		// Volume in m^3
		float32 V = ComputeVolume();
		
		// Inverse volume
		float32 invV = V > 0.0f ? 1.0f / V : 0.0f;

		// Apply pressure forces on particles
		for (u32 i = 0; i < m_mesh.triangleCount; ++i)
		{
			u32 i1 = m_mesh.triangles[i].v1;
			u32 i2 = m_mesh.triangles[i].v2;
			u32 i3 = m_mesh.triangles[i].v3;

			b3Particle* p1 = m_cloth->GetVertexParticle(i1);
			b3Particle* p2 = m_cloth->GetVertexParticle(i2);
			b3Particle* p3 = m_cloth->GetVertexParticle(i3);

			b3Vec3 v1 = p1->GetPosition();
			b3Vec3 v2 = p2->GetPosition();
			b3Vec3 v3 = p3->GetPosition();

			b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
			
			// Triangle area
			float32 A = n.Normalize();
			A *= 0.5f;

			// Ideal Gas Approximation

			// Number of gas moles
			const float32 k_n = 1.0f;
			
			// Ideal Gas Constant [J / K mol]
			const float32 k_R = 8.31f;
			
			// Gas temperature in Kelvin
			const float32 k_T = 100.0f;

			// Pressure in Pascals
			float32 P = invV * k_n * k_R * k_T;

			// Pressure vector
			b3Vec3 Pn = P * n;

			// Pressure force
			b3Vec3 FP = Pn * A;

			// Distribute
			p1->ApplyForce(FP);
			p2->ApplyForce(FP);
			p3->ApplyForce(FP);
		}
	}

	void Step()
	{
		Test::Step();

		ApplyPressureForces();

		m_cloth->Step(g_testSettings->inv_hertz);

		m_cloth->Draw();

		if (m_clothDragger->IsDragging())
		{
			b3Vec3 pA = m_clothDragger->GetPointA();
			b3Vec3 pB = m_clothDragger->GetPointB();

			g_draw->DrawPoint(pA, 2.0f, b3Color_green);

			g_draw->DrawPoint(pB, 2.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

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
		return new SoftBody();
	}

	b3QClothMesh m_mesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger;
};

#endif