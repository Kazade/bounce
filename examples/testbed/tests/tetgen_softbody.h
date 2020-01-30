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

#ifndef TETGEN_SOFTBODY_H
#define TETGEN_SOFTBODY_H

#include <fstream>

struct TetGenMesh : public b3SoftBodyMesh
{
	TetGenMesh()
	{
		vertexCount = 0;
		vertices = nullptr;
		triangleCount = 0;
		triangles = nullptr;
		tetrahedronCount = 0;
		tetrahedrons = nullptr;
	}

	~TetGenMesh()
	{
		free(vertices);
		free(triangles);
		free(tetrahedrons);
	}

	bool Load(const char* node_filename, const char* face_filename, const char* ele_filename)
	{
		{
			std::ifstream file(node_filename);
			if (!file.good())
			{
				printf("Could not open %s \n", node_filename);
				return false;
			}

			int nodeCount, nodeDimensions, attributeCount, boundaryMarkCount;

			std::string line;
			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				std::stringstream line_stream(line);

				line_stream >> nodeCount >> nodeDimensions >> attributeCount >> boundaryMarkCount;

				break;
			}

			if (nodeDimensions != 3)
			{
				printf(".node file: Only 3 dimensional nodes supported\n");
				return false;
			}

			if (attributeCount != 0)
			{
				printf(".node file: Only nodes with 0 attributes supported\n");
				return false;
			}

			if (boundaryMarkCount != 0)
			{
				printf(".node file: Only nodes with 0 markers supported\n");
				return false;
			}

			assert(vertexCount == 0);
			vertices = (b3Vec3*)malloc(sizeof(b3Vec3) * nodeCount);

			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				int nodeId;
				float x, y, z;

				std::stringstream line_stream(line);

				line_stream >> nodeId >> x >> y >> z;

				assert(nodeId > 0);
				assert(nodeId <= nodeCount);

				assert(b3IsValid(x));
				assert(b3IsValid(y));
				assert(b3IsValid(z));

				vertices[vertexCount].x = x;
				vertices[vertexCount].y = y;
				vertices[vertexCount].z = z;

				++vertexCount;
			}

			assert(vertexCount == nodeCount);
		}
		
		{
			std::ifstream file(face_filename);
			if (!file.good())
			{
				printf("Could not open %s \n", face_filename);
				return false;
			}

			int faceCount, boundaryMarkerCount;

			std::string line;
			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				std::stringstream line_stream(line);

				line_stream >> faceCount >> boundaryMarkerCount;

				break;
			}

			assert(triangleCount == 0);
			triangles = (b3SoftBodyMeshTriangle*)malloc(sizeof(b3SoftBodyMeshTriangle) * faceCount);

			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				int faceId;
				int v1, v2, v3;
				int corner;

				std::stringstream line_stream(line);

				line_stream >> faceId >> v1 >> v2 >> v3 >> corner;

				assert(faceId > 0);
				assert(faceId <= faceCount);

				// Make CCW
				b3Swap(v2, v3);

				triangles[triangleCount].v1 = u32(v1 - 1);
				triangles[triangleCount].v2 = u32(v2 - 1);
				triangles[triangleCount].v3 = u32(v3 - 1);

				++triangleCount;
			}

			assert(triangleCount == faceCount);
		}

		{
			std::ifstream file(ele_filename);
			if (!file.good())
			{
				printf("Could not open %s \n", ele_filename);
				return false;
			}

			int tetCount, nodesPerTet, attributeCount;

			std::string line;
			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				std::stringstream line_stream(line);

				line_stream >> tetCount >> nodesPerTet >> attributeCount;

				break;
			}

			if (nodesPerTet != 4)
			{
				printf(".ele file: Only 4 nodes per tetrahedran supported\n");
				return false;
			}

			if (attributeCount != 0)
			{
				printf(".ele file: Only elements with 0 attributes supported\n");
				return false;
			}

			assert(tetrahedronCount == 0);
			tetrahedrons = (b3SoftBodyMeshTetrahedron*)malloc(sizeof(b3SoftBodyMeshTetrahedron) * tetCount);

			while (std::getline(file, line))
			{
				if (line[0] == '#')
				{
					continue;
				}

				if (line[0] == 0)
				{
					continue;
				}

				int tetId;
				int v1, v2, v3, v4;

				std::stringstream line_stream(line);

				line_stream >> tetId >> v1 >> v2 >> v3 >> v4;

				assert(tetId > 0);
				assert(tetId <= tetCount);

				// Make CCW
				b3Swap(v2, v3);

				tetrahedrons[tetrahedronCount].v1 = u32(v1 - 1);
				tetrahedrons[tetrahedronCount].v2 = u32(v2 - 1);
				tetrahedrons[tetrahedronCount].v3 = u32(v3 - 1);
				tetrahedrons[tetrahedronCount].v4 = u32(v4 - 1);

				++tetrahedronCount;
			}

			assert(tetrahedronCount == tetCount);
		}

		return true;
	}
};

class TetGenSoftBody : public Test
{
public:
	TetGenSoftBody()
	{
		{
			bool ok = m_mesh.Load("data/octopus.node", "data/octopus.face", "data/octopus.ele");
			assert(ok);
		}

		for (u32 i = 0; i < m_mesh.vertexCount; ++i)
		{
			m_mesh.vertices[i].y += 10.0f;
		}

		// Create soft body
		b3SoftBodyDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.E = 1000.0f;
		def.nu = 0.3f;
		def.radius = 0.05f;
		def.friction = 0.2f;

		m_body = new b3SoftBody(def);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		// Create ground
		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			b3HullShape groundShape;
			groundShape.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &groundShape;
			sd.friction = 0.3f;

			b3Shape* s = b->CreateShape(sd);

			b3SoftBodyWorldShapeDef ssd;
			ssd.shape = s;

			m_body->CreateWorldShape(ssd); 
		}

		m_bodyDragger = new b3SoftBodyDragger(&m_ray, m_body);
	}

	~TetGenSoftBody()
	{
		delete m_bodyDragger;
		delete m_body;
	}

	void Step()
	{
		Test::Step();

		if (m_bodyDragger->IsDragging())
		{
			m_bodyDragger->Drag();
		}

		m_body->Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

		m_body->Draw();

		if (m_bodyDragger->IsDragging())
		{
			b3Vec3 pA = m_bodyDragger->GetPointA();
			b3Vec3 pB = m_bodyDragger->GetPointB();

			g_draw->DrawPoint(pA, 4.0f, b3Color_green);

			g_draw->DrawPoint(pB, 4.0f, b3Color_green);

			g_draw->DrawSegment(pA, pB, b3Color_white);
		}

		extern u32 b3_softBodySolverIterations;
		g_draw->DrawString(b3Color_white, "Iterations = %d", b3_softBodySolverIterations);

		scalar E = m_body->GetEnergy();
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_bodyDragger->IsDragging() == false)
		{
			m_bodyDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_bodyDragger->IsDragging() == true)
		{
			m_bodyDragger->StopDragging();
		}
	}

	static Test* Create()
	{
		return new TetGenSoftBody();
	}

	TetGenMesh m_mesh;
	b3SoftBody* m_body;
	b3SoftBodyDragger* m_bodyDragger;
};

#endif