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

#ifndef TABLE_CLOTH_H
#define TABLE_CLOTH_H

class TableCloth : public ClothTest
{
public:
	TableCloth() : m_rectangleGarment(5.0f, 5.0f)
	{
		// Generate 2D mesh
		m_rectangleGarmentMesh.Set(&m_rectangleGarment, 1.0f);

		// Create 3D mesh
		m_rectangleClothMesh.Set(&m_rectangleGarmentMesh);

		//  
		b3Mat33 dq = b3Mat33RotationX(0.5f * B3_PI);
		for (u32 i = 0; i < m_rectangleClothMesh.vertexCount; ++i)
		{
			m_rectangleClothMesh.vertices[i] = dq * m_rectangleClothMesh.vertices[i];
			m_rectangleClothMesh.vertices[i].y += 5.0f;
		}

		b3ClothDef def;
		def.mesh = &m_rectangleClothMesh;
		def.density = 0.2f;
		def.structural = 10000.0f;
		def.damping = 0.0f;

		m_cloth = m_world.CreateCloth(def);

		{
			b3BodyDef bd;
			bd.type = e_staticBody;

			b3Body* b = m_world.CreateBody(bd);

			m_tableHull.SetAsCylinder(5.0f, 2.0f);

			b3HullShape tableShape;
			tableShape.m_hull = &m_tableHull;
			tableShape.m_radius = 0.2f;

			//b3CapsuleShape tableShape;
			//tableShape.m_centers[0].Set(0.0f, 0.0f, -1.0f);
			//tableShape.m_centers[1].Set(0.0f, 0.0f, 1.0f);
			//tableShape.m_radius = 2.0f;

			b3ShapeDef sd;
			sd.shape = &tableShape;
			sd.friction = 1.0f;

			b->CreateShape(sd);
		}
	}

	static Test* Create()
	{
		return new TableCloth();
	}

	b3RectangleGarment m_rectangleGarment;
	b3GarmentMesh m_rectangleGarmentMesh;
	b3GarmentClothMesh m_rectangleClothMesh;
	
	b3QHull m_tableHull;
};

#endif