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

#ifndef MESH_TEST_H
#define MESH_TEST_H

class MeshContactTest : public Test
{
public:
	MeshContactTest()
	{
		m_gridMesh.BuildTree();
		m_gridMesh.BuildAdjacency();

		// Transform grid into a terrain
		for (u32 i = 0; i < m_terrainMesh.vertexCount; ++i)
		{
			m_terrainMesh.vertices[i].y = RandomFloat(0.0f, 1.0f);
		}

		m_terrainMesh.BuildTree();
		m_terrainMesh.BuildAdjacency();

		{
			b3BodyDef bd;
			b3Body* groundBody = m_world.CreateBody(bd);

			b3MeshShape ms;
			ms.m_mesh = &m_gridMesh;
			ms.m_scale.Set(2.0f, 1.0f, 2.0f);

			b3ShapeDef sd;
			sd.shape = &ms;

			m_groundShape = (b3MeshShape*)groundBody->CreateShape(sd);
			
			m_selection = m_groundShape->m_mesh->triangleCount / 2;
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyShape = body->CreateShape(sd);
			}
		}

		m_drawEdgeTypes = true;
	}

	void KeyDown(int key)
	{
		u32 minSelection = 0;
		if (key == GLFW_KEY_LEFT)
		{
			m_selection = m_selection == minSelection ? minSelection : m_selection - 1;
		}

		u32 maxSelection = m_groundShape->m_mesh->triangleCount - 1;
		if (key == GLFW_KEY_RIGHT)
		{
			m_selection = m_selection == maxSelection ? maxSelection : m_selection + 1;
		}

		if (key == GLFW_KEY_E)
		{
			m_drawEdgeTypes = !m_drawEdgeTypes;
		}

		if (key == GLFW_KEY_S || key == GLFW_KEY_C || key == GLFW_KEY_H)
		{
			b3Body* body = m_bodyShape->GetBody();

			m_world.DestroyBody(body);

			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			body = m_world.CreateBody(bd);

			if (key == GLFW_KEY_S)
			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyShape = body->CreateShape(sd);
			}

			if (key == GLFW_KEY_C)
			{
				b3CapsuleShape capsule;
				capsule.m_vertex1.Set(0.0f, -1.0f, 0.0f);
				capsule.m_vertex2.Set(0.0f, 1.0f, 0.0f);
				capsule.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &capsule;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyShape = body->CreateShape(sd);
			}

			if (key == GLFW_KEY_H)
			{
				b3HullShape hull;
				hull.m_hull = &b3BoxHull_identity;

				b3ShapeDef sd;
				sd.shape = &hull;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyShape = body->CreateShape(sd);
			}
		}

		if (key == GLFW_KEY_G || key == GLFW_KEY_T)
		{
			b3Body* groundBody = m_groundShape->GetBody();
			m_world.DestroyBody(groundBody);

			b3BodyDef bd;
			groundBody = m_world.CreateBody(bd);

			if (key == GLFW_KEY_G)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_gridMesh;
				ms.m_scale.Set(2.0f, 1.0f, 2.0f);

				b3ShapeDef sd;
				sd.shape = &ms;

				m_groundShape = (b3MeshShape*)groundBody->CreateShape(sd);
			}

			if (key == GLFW_KEY_T)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_terrainMesh;
				ms.m_scale.Set(2.0f, 1.5f, 2.0f);

				b3ShapeDef sd;
				sd.shape = &ms;

				m_groundShape = (b3MeshShape*)groundBody->CreateShape(sd);
			}
			
			m_selection = m_groundShape->m_mesh->triangleCount / 2;
		}
	}

	void Step()
	{
		Test::Step();

		const b3Mesh* mesh = m_groundShape->m_mesh;
		b3Vec3 scale = m_groundShape->m_scale;
		b3Body* body = m_groundShape->GetBody();
		b3Transform xf = body->GetTransform();

		{
			const b3MeshTriangle* triangle = mesh->triangles + m_selection;
			const b3MeshTriangleWings* triangleWings = mesh->triangleWings + m_selection;

			for (u32 i = 0; i < 3; ++i)
			{
				u32 j = i + 1 < 3 ? i + 1 : 0;

				u32 v1 = triangle->GetVertex(i);
				u32 v2 = triangle->GetVertex(j);

				b3Vec3 p1 = xf * b3MulCW(scale, mesh->vertices[v1]);
				b3Vec3 p2 = xf * b3MulCW(scale, mesh->vertices[v2]);

				b3Vec3 center = scalar(0.5) * (p1 + p2);
				g_draw->DrawString(b3Color_white, center, "e%d", i);

				u32 wingVertex = triangleWings->GetVertex(i);

				if (wingVertex != B3_NULL_VERTEX)
				{
					b3Vec3 vertex = xf * b3MulCW(scale, mesh->vertices[wingVertex]);
					g_draw->DrawString(b3Color_white, vertex, "u%d", i);
				}
			}
		}

		if (m_drawEdgeTypes)
		{
			b3Vec3 eyePoint(0.0f, 10.0f, 0.0f);

			for (u32 i = 0; i < mesh->triangleCount; ++i)
			{
				b3MeshTriangle* triangle = mesh->triangles + i;
				b3MeshTriangleWings* triangleWings = mesh->triangleWings + i;

				b3Vec3 A = xf * b3MulCW(scale, mesh->vertices[triangle->v1]);
				b3Vec3 B = xf * b3MulCW(scale, mesh->vertices[triangle->v2]);
				b3Vec3 C = xf * b3MulCW(scale, mesh->vertices[triangle->v3]);

				b3Vec3 N = b3Cross(B - A, C - A);
				N.Normalize();

				b3Plane plane(N, A);
				if (b3Distance(eyePoint, plane) < 0.0f)
				{
					plane = b3Plane(-N, A);
				}

				for (u32 j = 0; j < 3; ++j)
				{
					u32 k = j + 1 < 3 ? j + 1 : 0;

					u32 v1 = triangle->GetVertex(j);
					u32 v2 = triangle->GetVertex(k);

					u32 u = triangleWings->GetVertex(j);

					b3Vec3 p1 = xf * b3MulCW(scale, mesh->vertices[v1]);
					b3Vec3 p2 = xf * b3MulCW(scale, mesh->vertices[v2]);

					b3Vec3 center = scalar(0.5) * (p1 + p2);

					if (u == B3_NULL_VERTEX)
					{
						g_draw->DrawPoint(center, scalar(4), b3Color_white);
						continue;
					}

					b3Vec3 wingVertex = xf * b3MulCW(scale, mesh->vertices[u]);

					scalar d = b3Distance(wingVertex, plane);

					const scalar kCoplanarTol = 0.005f;

					if (d < -kCoplanarTol)
					{
						// Below <=> Convex
						g_draw->DrawPoint(center, scalar(4), b3Color_green);
					}
					else if (d > kCoplanarTol)
					{
						// Above <=> Concave
						g_draw->DrawPoint(center, scalar(4), b3Color_yellow);
					}
					else
					{
						// d > -e && d < e
						// On <=> Coplanar
						g_draw->DrawPoint(center, scalar(4), b3Color_red);
					}
				}
			}
		}

		g_draw->DrawString(b3Color_white, "E - View Edge Types");
		g_draw->DrawString(b3Color_white, "Arrows - Select Face Wings");
		g_draw->DrawString(b3Color_white, "S - Sphere");
		g_draw->DrawString(b3Color_white, "C - Capsule");
		g_draw->DrawString(b3Color_white, "H - Hull");
		g_draw->DrawString(b3Color_white, "G - Grid");
		g_draw->DrawString(b3Color_white, "T - Terrain");
	}

	static Test* Create()
	{
		return new MeshContactTest();
	}

	bool m_drawEdgeTypes;
	u32 m_selection;

	b3GridMesh<25, 25> m_terrainMesh;
	b3GridMesh<25, 25> m_gridMesh;

	b3MeshShape* m_groundShape;
	b3Shape* m_bodyShape;
};

#endif