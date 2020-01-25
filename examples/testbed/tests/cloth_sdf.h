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

#ifndef CLOTH_SDF_H
#define CLOTH_SDF_H

#define TINYOBJLOADER_IMPLEMENTATION 
#include <tinyobjloader/tiny_obj_loader.h>

struct SDFMesh
{
	u32 vertexCount;
	b3Vec3* vertices;
	u32 indexCount;
	u32* indices;

	SDFMesh()
	{
		vertexCount = 0;
		vertices = nullptr;
		indexCount = 0;
		indices = nullptr;
	}

	~SDFMesh()
	{
		free(vertices);
		free(indices);
	}

	bool Load(const char* filename)
	{
		tinyobj::attrib_t attributes;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		std::string warning;
		std::string error;
		bool ok = tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error, filename);
		if (!ok)
		{
			return false;
		}
		
		assert(vertexCount == 0);
		vertexCount = attributes.vertices.size() / 3;
		vertices = (b3Vec3*)malloc(vertexCount * sizeof(b3Vec3));
		for (size_t i = 0; i < attributes.vertices.size() / 3; ++i)
		{
			tinyobj::real_t x = attributes.vertices[3 * i + 0];
			tinyobj::real_t y = attributes.vertices[3 * i + 1];
			tinyobj::real_t z = attributes.vertices[3 * i + 2];

			b3Vec3 v(x, y, z);

			vertices[i] = v;
		}

		assert(indexCount == 0);
		for (size_t s = 0; s < shapes.size(); s++)
		{
			tinyobj::shape_t& shape = shapes[s];
			for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
			{
				indexCount += 3;
			}
		}
		indices = (u32*)malloc(indexCount * sizeof(u32));

		indexCount = 0;
		for (size_t s = 0; s < shapes.size(); s++) 
		{
			tinyobj::shape_t& shape = shapes[s];

			size_t index_offset = 0;
			for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) 
			{
				unsigned char fv = shapes[s].mesh.num_face_vertices[f];

				for (size_t v = 0; v < 3; v++) 
				{
					tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
					
					size_t vi = idx.vertex_index;

					indices[indexCount++] = vi;
				}
				
				index_offset += fv;
			}
		}
		
		return true;
	}

	void Draw(const b3Transform& xf, const b3Vec3& scale, const b3Color& color) const
	{
		for (u32 i = 0; i < indexCount / 3; ++i)
		{
			u32 i1 = indices[3 * i + 0];
			u32 i2 = indices[3 * i + 1];
			u32 i3 = indices[3 * i + 2];

			b3Vec3 v1 = xf * b3MulCW(scale, vertices[i1]);
			b3Vec3 v2 = xf * b3MulCW(scale, vertices[i2]);
			b3Vec3 v3 = xf * b3MulCW(scale, vertices[i3]);

			b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
			n.Normalize();

			g_draw->DrawSolidTriangle(n, v1, v2, v3, color);
		}
	}
};

class ClothSDF : public Test
{
public:
	ClothSDF()
	{
		// Translate the cloth mesh
		for (u32 i = 0; i < m_clothMesh.vertexCount; ++i)
		{
			m_clothMesh.vertices[i].y += 5.0f;
		}

		// Create cloth
		b3ClothDef def;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.streching = 10000.0f;
		def.strechDamping = 100.0f;
		def.thickness = 0.2f;
		def.friction = 0.2f;

		m_cloth = new b3Cloth(def);

		m_cloth->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		{
			bool ok = m_sdfMesh.Load("data/teapot.obj");
			assert(ok);
		}

		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			bool ok = m_sdf.Load("data/teapot.cdf");
			assert(ok);

			b3SDFShape sdfShape;
			sdfShape.m_sdf = &m_sdf;
			sdfShape.m_radius = 0.2f;

			b3ShapeDef sd;
			sd.shape = &sdfShape;
			sd.friction = 1.0f;
			
			m_sdfShape = (b3SDFShape*)b->CreateShape(sd);

			b3ClothWorldShapeDef csd;
			csd.shape = m_sdfShape;

			m_cloth->CreateWorldShape(csd);
		}

		m_clothDragger = new b3ClothDragger(&m_ray, m_cloth);
	}

	~ClothSDF()
	{
		delete m_clothDragger;
		delete m_cloth;
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

		b3Body* sdfBody = m_sdfShape->GetBody();
		m_sdfMesh.Draw(sdfBody->GetTransform(), m_sdfShape->m_scale, b3Color_white);

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

	static Test* Create()
	{
		return new ClothSDF();
	}

	b3GridClothMesh<10, 10> m_clothMesh;
	b3Cloth* m_cloth;
	b3ClothDragger* m_clothDragger;

	SDFMesh m_sdfMesh;
	b3SDF m_sdf;
	b3SDFShape* m_sdfShape;
};

#endif