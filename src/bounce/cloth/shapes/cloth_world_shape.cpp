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

#include <bounce/cloth/shapes/cloth_world_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3ClothWorldShape::b3ClothWorldShape(const b3ClothWorldShapeDef& def, b3Cloth* cloth)
{
	m_type = e_clothWorldShape;
	m_shape = def.shape;
	m_cloth = cloth;
}

b3ClothWorldShape::~b3ClothWorldShape()
{

}

b3AABB b3ClothWorldShape::ComputeAABB() const
{
	const b3Body* b = m_shape->GetBody();
	b3Transform xf = b->GetTransform();
	b3AABB aabb;
	m_shape->ComputeAABB(&aabb, xf);
	return aabb;
}

void b3ClothWorldShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_cloth->m_contactManager.m_broadPhase.MoveProxy(m_broadPhaseId, aabb, displacement);
}

void b3ClothWorldShape::DestroyContacts()
{
	b3ClothSphereAndShapeContact* c = m_cloth->m_contactManager.m_sphereAndShapeContactList.m_head;
	while (c)
	{
		b3ClothSphereAndShapeContact* c0 = c;
		c = c->m_next;

		if (c0->m_s2 == this)
		{
			m_cloth->m_contactManager.Destroy(c0);
		}
	}
}