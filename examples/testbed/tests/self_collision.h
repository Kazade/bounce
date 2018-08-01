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

#ifndef SELF_COLLISION_H
#define SELF_COLLISION_H

class SelfCollision : public ClothTest
{
public:
	SelfCollision() : m_rectangleGarment(5.0f, 5.0f)
	{
		// Generate 2D mesh
		m_rectangleGarmentMesh.Set(&m_rectangleGarment, 1.0f);

		// Create 3D mesh
		m_rectangleClothMesh.Set(&m_rectangleGarmentMesh);

		b3Mat33 Rx = b3Mat33RotationX(0.5f * B3_PI);
		for (u32 i = 0; i < m_rectangleClothMesh.vertexCount; ++i)
		{
			m_rectangleClothMesh.vertices[i] = Rx * m_rectangleClothMesh.vertices[i];
			m_rectangleClothMesh.vertices[i].y += 5.0f;
		}

		b3ClothDef def;
		def.mesh = &m_rectangleClothMesh;
		def.density = 1.0f;
		def.structural = 100000.0f;
		
		m_cloth = m_world.CreateCloth(def);

		for (b3Particle* p = m_cloth->GetParticleList().m_head; p; p = p->GetNext())
		{
			p->SetRadius(0.2f);
			p->SetFriction(0.2f);
		}

		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			b3CapsuleShape capsuleShape;
			capsuleShape.m_centers[0].Set(0.0f, 0.0f, -5.0f);
			capsuleShape.m_centers[1].Set(0.0f, 0.0f, 5.0f);
			capsuleShape.m_radius = 1.0f;;

			b3ShapeDef sd;
			sd.shape = &capsuleShape;
			sd.friction = 1.0f;

			b->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new SelfCollision();
	}

	b3RectangleGarment m_rectangleGarment;
	b3GarmentMesh m_rectangleGarmentMesh;
	b3GarmentClothMesh m_rectangleClothMesh;

	b3BoxHull m_boxHull;
};

#endif