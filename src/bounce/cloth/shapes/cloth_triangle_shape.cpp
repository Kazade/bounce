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

#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/cloth.h>

b3ClothTriangleShape::b3ClothTriangleShape(const b3ClothTriangleShapeDef& def, b3Cloth* cloth)
{
	m_type = e_clothTriangleShape;
	m_cloth = cloth;
	m_p1 = def.p1;
	m_p2 = def.p2;
	m_p3 = def.p3;
	m_area = b3Area(def.v1, def.v2, def.v3);
}

b3ClothTriangleShape::~b3ClothTriangleShape()
{

}

b3AABB b3ClothTriangleShape::ComputeAABB() const
{
	b3AABB aabb;
	aabb.SetTriangle(m_p1->m_position, m_p2->m_position, m_p3->m_position);
	aabb.Extend(m_radius);
	return aabb;
}

void b3ClothTriangleShape::DestroyContacts()
{
	{
		// Destroy sphere contacts
		b3ClothSphereAndTriangleContact* c = m_cloth->m_contactManager.m_sphereAndTriangleContactList.m_head;
		while (c)
		{
			if (c->m_s2 == this)
			{
				b3ClothSphereAndTriangleContact* quack = c;
				c = c->m_next;
				m_cloth->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}
}

void b3ClothTriangleShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_cloth->m_contactManager.m_broadPhase.MoveProxy(m_broadPhaseId, aabb, displacement);
}