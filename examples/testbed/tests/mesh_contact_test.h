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

#ifndef MESH_TEST_H
#define MESH_TEST_H

class MeshContactTest : public Test
{
public:
	MeshContactTest()
	{
		// Transform grid into a terrain
		for (u32 i = 0; i < m_terrainMesh.vertexCount; ++i)
		{
			m_terrainMesh.vertices[i].y = RandomFloat(0.0f, 1.0f);
		}

		m_terrainMesh.BuildTree();

		{
			b3BodyDef bd;
			m_ground = m_world.CreateBody(bd);

			b3MeshShape ms;
			ms.m_mesh = &m_groundMesh;

			b3ShapeDef sd;
			sd.shape = &ms;

			m_ground->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			m_body = m_world.CreateBody(bd);
			
			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_body->CreateShape(sd);
			}
		}
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_S || key == GLFW_KEY_C || key == GLFW_KEY_H)
		{
			if (m_body)
			{
				m_world.DestroyBody(m_body);
			}

			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			m_body = m_world.CreateBody(bd);

			if (key == GLFW_KEY_S)
			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_body->CreateShape(sd);
			}

			if (key == GLFW_KEY_C)
			{
				b3CapsuleShape capsule;
				capsule.m_centers[0].Set(0.0f, -1.0f, 0.0f);
				capsule.m_centers[1].Set(0.0f, 1.0f, 0.0f);
				capsule.m_radius = 1.0f;

				b3ShapeDef sd;
				sd.shape = &capsule;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_body->CreateShape(sd);
			}

			if (key == GLFW_KEY_H)
			{
				b3HullShape hull;
				hull.m_hull = &m_boxHull;

				b3ShapeDef sd;
				sd.shape = &hull;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_body->CreateShape(sd);
			}
		}

		if (key == GLFW_KEY_G || key == GLFW_KEY_T)
		{
			if (m_ground)
			{
				m_world.DestroyBody(m_ground);
			}

			b3BodyDef bd;
			m_ground = m_world.CreateBody(bd);

			if (key == GLFW_KEY_G)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_groundMesh;

				b3ShapeDef sd;
				sd.shape = &ms;

				m_ground->CreateShape(sd);
			}

			if (key == GLFW_KEY_T)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_terrainMesh;

				b3ShapeDef sd;
				sd.shape = &ms;

				m_ground->CreateShape(sd);
			}
		}
	}

	void Step()
	{
		Test::Step();

		b3Color color(1.0f, 1.0f, 1.0f);
		
		g_debugDraw->DrawString("S - Sphere", color);
		g_debugDraw->DrawString("C - Capsule", color);
		g_debugDraw->DrawString("H - Hull", color);
		g_debugDraw->DrawString("G - Grid", color);
		g_debugDraw->DrawString("T - Terrain", color);
	}

	static Test* Create()
	{
		return new MeshContactTest();
	}

	b3GridMesh<25, 25> m_terrainMesh;
	
	b3Body* m_ground;
	b3Body* m_body;
};

#endif