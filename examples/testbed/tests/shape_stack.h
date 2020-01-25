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

#ifndef SHAPE_STACK_H
#define SHAPE_STACK_H

class ShapeStack : public Test
{
public:
	ShapeStack()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3MeshShape ms;
			ms.m_mesh = &m_groundMesh;
			
			b3ShapeDef sd;
			sd.shape = &ms;
			
			ground->CreateShape(sd);
		}


		for (scalar y = 2.5f; y < 20.0f; y += 2.5f)
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(-10.0f, y, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape sphere;
			sphere.m_center.SetZero();
			sphere.m_radius = 1.0f;

			b3ShapeDef sdef;
			sdef.shape = &sphere;
			sdef.density = 1.0f;
			sdef.friction = 0.3f;
			
			body->CreateShape(sdef);
		}

		for (scalar y = 2.5f; y < 20.0f; y += 2.5f)
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, y, 0.0f);
			bd.orientation = b3QuatRotationZ(0.5f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3CapsuleShape capsule;
			capsule.m_vertex1.Set(0.0f, -1.0f, 0.0f);
			capsule.m_vertex2.Set(0.0f, 1.0f, 0.0f);
			capsule.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &capsule;
			sd.density = 1.0f;
			sd.friction = 0.3f;
			
			body->CreateShape(sd);
		}
		
		for (scalar y = 2.5f; y < 20.0f; y += 2.5f)
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(10.0f, y, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hull;
			hull.m_hull = &b3BoxHull_identity;
			
			b3ShapeDef sd;
			sd.shape = &hull;
			sd.density = 1.0f;
			sd.friction = 0.3f;
			
			body->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new ShapeStack();
	}
};

#endif
