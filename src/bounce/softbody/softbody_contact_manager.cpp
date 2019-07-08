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
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_listeners.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3SoftBodyContactManager::b3SoftBodyContactManager() : 
	m_nodeBodyContactBlocks(sizeof(b3NodeBodyContact))
{

}

class b3SoftBodyContactManagerFindNewBodyContactsQueryListener : public b3QueryListener
{
public:
	virtual bool ReportShape(b3Shape* s2)
	{
		cm->AddNSPair(n1, s2);

		// Keep looking for overlaps
		return true;
	}

	b3SoftBodyContactManager* cm;
	b3SoftBodyNode* n1;
};

void b3SoftBodyContactManager::FindNewBodyContacts()
{
	B3_PROFILE("Soft Body Find New Body Contacts");

	// Is there a world attached to this body?
	if (m_body->m_world == nullptr)
	{
		return;
	}

	for (u32 i = 0; i < m_body->m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_body->m_nodes + i;

		if (n->m_type != e_dynamicSoftBodyNode)
		{
			continue;
		}

		b3AABB3 aabb = m_broadPhase.GetAABB(n->m_broadPhaseId);

		b3SoftBodyContactManagerFindNewBodyContactsQueryListener listener;
		listener.cm = this;
		listener.n1 = n;

		m_body->m_world->QueryAABB(&listener, aabb);
	}
}

void b3SoftBodyContactManager::AddNSPair(b3SoftBodyNode* n1, b3Shape* s2)
{
	// Check if there is a contact between the two entities.
	for (b3NodeBodyContact* c = m_nodeBodyContactList.m_head; c; c = c->m_next)
	{
		if (c->m_n1 == n1 && c->m_s2 == s2)
		{
			// A contact already exists.
			return;
		}
	}

	bool isntDynamic1 = n1->m_type != e_dynamicSoftBodyNode;
	bool isntDynamic2 = s2->GetBody()->GetType() != e_dynamicBody;

	if (isntDynamic1 && isntDynamic2)
	{
		// The entities must not collide with each other.
		return;
	}

	// Create a new contact.
	b3NodeBodyContact* c = CreateNodeBodyContact();

	c->m_n1 = n1;
	c->m_s2 = s2;
	c->m_active = false;
	c->m_normalImpulse = 0.0f;
	c->m_tangentImpulse.SetZero();

	// Add the contact to the body contact list.
	m_nodeBodyContactList.PushFront(c);
}

void b3SoftBodyContactManager::UpdateBodyContacts()
{
	B3_PROFILE("Soft Body Update Body Contacts");

	// Update the state of node-body contacts.
	b3NodeBodyContact* c = m_nodeBodyContactList.m_head;
	while (c)
	{
		bool isntDynamic1 = c->m_n1->m_type != e_dynamicSoftBodyNode;
		bool isntDynamic2 = c->m_s2->GetBody()->GetType() != e_dynamicBody;

		// Cease the contact if entities must not collide with each other.
		if (isntDynamic1 && isntDynamic2)
		{
			b3NodeBodyContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		b3AABB3 aabb1 = m_broadPhase.GetAABB(c->m_n1->m_broadPhaseId);
		b3AABB3 aabb2 = c->m_s2->GetAABB();

		// Destroy the contact if entities AABBs are not overlapping.
		bool overlap = b3TestOverlap(aabb1, aabb2);
		if (overlap == false)
		{
			b3NodeBodyContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update();

		c = c->m_next;
	}
}

b3NodeBodyContact* b3SoftBodyContactManager::CreateNodeBodyContact()
{
	void* block = m_nodeBodyContactBlocks.Allocate();
	return new(block) b3NodeBodyContact();
}

void b3SoftBodyContactManager::Destroy(b3NodeBodyContact* c)
{
	m_nodeBodyContactList.Remove(c);

	c->~b3NodeBodyContact();

	m_nodeBodyContactBlocks.Free(c);
}