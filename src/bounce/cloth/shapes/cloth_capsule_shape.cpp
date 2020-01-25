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

#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/cloth.h>

b3ClothCapsuleShape::b3ClothCapsuleShape(const b3ClothCapsuleShapeDef& def, b3Cloth* cloth)
{
	m_type = e_clothCapsuleShape;
	m_cloth = cloth;
	m_p1 = def.p1;
	m_p2 = def.p2;
}

b3ClothCapsuleShape::~b3ClothCapsuleShape()
{

}

void b3ClothCapsuleShape::DestroyContacts()
{
	b3ClothCapsuleAndCapsuleContact* c = m_cloth->m_contactManager.m_capsuleAndCapsuleContactList.m_head;
	while (c)
	{
		if (c->m_s1 == this || c->m_s2 == this)
		{
			b3ClothCapsuleAndCapsuleContact* quack = c;
			c = c->m_next;
			m_cloth->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}

b3AABB b3ClothCapsuleShape::ComputeAABB() const
{
	b3AABB aabb;
	aabb.SetSegment(m_p1->m_position, m_p2->m_position);
	aabb.Extend(m_radius);
	return aabb;
}

void b3ClothCapsuleShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_cloth->m_contactManager.m_broadPhase.MoveProxy(m_broadPhaseId, aabb, displacement);
}