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

#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody.h>
#include <bounce/softbody/shapes/softbody_sphere_shape.h>

void b3SoftBodyNode::SetType(b3SoftBodyNodeType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;
	m_force.SetZero();

	if (type == e_staticSoftBodyNode)
	{
		m_velocity.SetZero();
		SynchronizeSpheres();
	}

	DestroyContacts();
}

void b3SoftBodyNode::SynchronizeSpheres()
{
	for (b3SoftBodySphereShape* s = m_body->m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_node == this)
		{
			s->Synchronize(b3Vec3_zero);
		}
	}
}

void b3SoftBodyNode::DestroyContacts()
{
	b3SoftBodySphereAndShapeContact* c = m_body->m_contactManager.m_sphereAndShapeContactList.m_head;
	while (c)
	{
		if (c->m_s1->m_node == this)
		{
			b3SoftBodySphereAndShapeContact* quack = c;
			c = c->m_next;
			m_body->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}