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

#ifndef SPRING_CLOTH_H
#define SPRING_CLOTH_H

class SpringCloth : public SpringClothTest
{
public:
	SpringCloth() 
	{
		b3SpringClothDef def;
		def.allocator = &m_clothAllocator;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.ks = 100000.0f;
		def.gravity.Set(0.0f, -10.0f, 0.0f);

		m_cloth.Initialize(def);

		b3AABB3 aabb;
		aabb.m_lower.Set(-5.0f, -1.0f, -6.0f);
		aabb.m_upper.Set(5.0f, 1.0f, -4.0f);

		for (u32 i = 0; i < def.mesh->vertexCount; ++i)
		{
			if (aabb.Contains(def.mesh->vertices[i]))
			{
				m_cloth.SetType(i, b3MassType::e_staticMass);
			}
		}		
	}

	static Test* Create()
	{
		return new SpringCloth();
	}

	b3GridMesh<10, 10> m_clothMesh;
};

#endif