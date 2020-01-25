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

#include <bounce/softbody/softbody_contact_manager.h>
#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/shapes/softbody_sphere_shape.h>
#include <bounce/softbody/shapes/softbody_world_shape.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3SoftBodyContactManager::b3SoftBodyContactManager() : 
	m_sphereAndShapeContactBlocks(sizeof(b3SoftBodySphereAndShapeContact))
{

}

void b3SoftBodyContactManager::FindNewContacts()
{
	B3_PROFILE("Soft Body Find New Contacts");

	m_broadPhase.FindPairs(this);
}

void b3SoftBodyContactManager::AddPair(void* data1, void* data2)
{
	b3SoftBodyShape* shape1 = (b3SoftBodyShape*)data1;
	b3SoftBodyShape* shape2 = (b3SoftBodyShape*)data2;

	if (shape1->m_type > shape2->m_type)
	{
		b3Swap(shape1, shape2);
	}

	if (shape1->m_type == e_softBodySphereShape && shape2->m_type == e_softBodyWorldShape)
	{
		b3SoftBodySphereShape* s1 = (b3SoftBodySphereShape*)shape1;
		b3SoftBodyWorldShape* ws2 = (b3SoftBodyWorldShape*)shape2;

		b3SoftBodyNode* n1 = s1->m_node;

		const b3Shape* s2 = ws2->m_shape;
		const b3Body* b2 = s2->GetBody();

		if (b2->GetType() != e_staticBody)
		{
			// Only collisions with static bodies are supported.
			return;
		}

		if (n1->m_type != e_dynamicSoftBodyNode)
		{
			// The entities must not collide with each other.
			return;
		}

		// Check if there is a contact between the two entities.
		for (b3SoftBodySphereAndShapeContact* c = m_sphereAndShapeContactList.m_head; c; c = c->m_next)
		{
			if (c->m_s1 == s1 && c->m_s2 == ws2)
			{
				// A contact already exists.
				return;
			}
		}

		// Create a new contact.
		b3SoftBodySphereAndShapeContact* c = CreateSphereAndShapeContact();

		c->m_s1 = s1;
		c->m_s2 = ws2;
		c->m_active = false;
		c->m_normalImpulse = scalar(0);
		c->m_tangentImpulse.SetZero();

		// Add the contact to the soft body contact list.
		m_sphereAndShapeContactList.PushFront(c);
	}
}

void b3SoftBodyContactManager::UpdateContacts()
{
	B3_PROFILE("Soft Body Update Contacts");

	{
		// Update the state of sphere and shape contacts.
		b3SoftBodySphereAndShapeContact* c = m_sphereAndShapeContactList.m_head;
		while (c)
		{
			b3SoftBodySphereShape* s1 = c->m_s1;
			b3SoftBodyNode* n1 = s1->m_node;

			b3SoftBodyWorldShape* ws2 = c->m_s2;
			const b3Shape* s2 = ws2->m_shape;
			const b3Body* b2 = s2->GetBody();

			bool isntDynamic1 = n1->GetType() != e_dynamicSoftBodyNode;
			bool isntDynamic2 = b2->GetType() != e_dynamicBody;

			// Cease the contact if entities must not collide with each other.
			if (isntDynamic1 && isntDynamic2)
			{
				b3SoftBodySphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_broadPhaseId;
			u32 proxy2 = c->m_s2->m_broadPhaseId;

			// Destroy the contact if primitive AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3SoftBodySphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}
}

b3SoftBodySphereAndShapeContact* b3SoftBodyContactManager::CreateSphereAndShapeContact()
{
	void* block = m_sphereAndShapeContactBlocks.Allocate();
	return new(block) b3SoftBodySphereAndShapeContact();
}

void b3SoftBodyContactManager::Destroy(b3SoftBodySphereAndShapeContact* c)
{
	m_sphereAndShapeContactList.Remove(c);
	c->~b3SoftBodySphereAndShapeContact();
	m_sphereAndShapeContactBlocks.Free(c);
}