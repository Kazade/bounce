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

#include <bounce/dynamics/contact_manager.h>
#include <bounce/dynamics/contacts/convex_contact.h>
#include <bounce/dynamics/contacts/mesh_contact.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world_listeners.h>

b3ContactManager::b3ContactManager() : 
	m_convexBlocks(sizeof(b3ConvexContact)),
	m_meshBlocks(sizeof(b3MeshContact))
{
	m_contactListener = NULL;
	m_contactFilter = NULL;
}

void b3ContactManager::AddPair(void* dataA, void* dataB) 
{
	b3Shape* shapeA = (b3Shape*)dataA;
	b3Shape* shapeB = (b3Shape*)dataB;

	b3Body* bodyA = shapeA->GetBody();
	b3Body* bodyB = shapeB->GetBody();
	
	if (bodyA == bodyB) 
	{
		// Two shapes that belong to the same body cannot collide.
		return;
	}

	// Check if there is a contact between the two shapes.
	// Search the list A or B. The shorter if possible.
	for (b3ContactEdge* ce = shapeB->m_contactEdges.m_head; ce; ce = ce->m_next)
	{
		if (ce->other == shapeA)
		{
			b3Contact* c = ce->contact;
			
			b3Shape* sA = c->GetShapeA();
			b3Shape* sB = c->GetShapeB();

			if (sA == shapeA && sB == shapeB)
			{
				// A contact already exists.
				return;
			}

			if (sB == shapeA && sA == shapeB)
			{
				// A contact already exists.
				return;
			}
		}
	}

	// Check if a joint prevents collision between the bodies.
	if (bodyA->ShouldCollide(bodyB) == false)
	{
		// The bodies must not collide with each other.
		return;
	}

	// Check if the contact filter prevents the collision.
	if (m_contactFilter)
	{
		if (m_contactFilter->ShouldCollide(shapeA, shapeB) == false)
		{
			return;
		}
	}

	// Create contact.
	b3Contact* c = Create(shapeA, shapeB);
	if (c == NULL)
	{
		return;
	}

	// Get the shapes from the contact again
	// because contact creation will swap the shapes if typeA > typeB.
	shapeA = c->GetShapeA();
	shapeB = c->GetShapeB();
	bodyA = shapeA->GetBody();
	bodyB = shapeB->GetBody();

	c->m_flags = 0;
	b3OverlappingPair* pair = &c->m_pair;

	// Initialize edge A
	pair->edgeA.contact = c;
	pair->edgeA.other = shapeB;
	
	// Add edge A to shape A's contact list.
	shapeA->m_contactEdges.PushFront(&pair->edgeA);

	// Initialize edge B
	pair->edgeB.contact = c;
	pair->edgeB.other = shapeA;

	// Add edge B to shape B's contact list.
	shapeB->m_contactEdges.PushFront(&pair->edgeB);

	// Awake the bodies if both are not sensors.
	if (!shapeA->IsSensor() && !shapeB->IsSensor()) 
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}

	// Add the contact to the world contact list.
	m_contactList.PushFront(c);
	
	if (c->m_type == e_meshContact)
	{
		// Add the contact to the world mesh contact list.
		b3MeshContact* mc = (b3MeshContact*)c;
		
		// Find new shape-child overlapping pairs.
		mc->FindNewPairs();

		b3MeshContactLink* link = &mc->m_link;
		link->m_c = mc;
		m_meshContactList.PushFront(link);
	}
}

void b3ContactManager::SynchronizeShapes()
{
	b3MeshContactLink* c = m_meshContactList.m_head;
	while (c)
	{
		c->m_c->SynchronizeShapes();
		c = c->m_next;
	}
}

// Find potentially overlapping shape pairs.
void b3ContactManager::FindNewContacts()
{
	m_broadPhase.FindPairs(this);

	b3MeshContactLink* c = m_meshContactList.m_head;
	while (c)
	{
		c->m_c->FindNewPairs();
		c = c->m_next;
	}
}

void b3ContactManager::UpdateContacts() 
{	
	B3_PROFILE("Update Contacts");
	
	// Update the state of all contacts.
	b3Contact* c = m_contactList.m_head;
	while (c)
	{
		b3OverlappingPair* pair = &c->m_pair;

		b3Shape* shapeA = pair->shapeA;
		u32 proxyA = shapeA->m_broadPhaseID;
		b3Body* bodyA = shapeA->m_body;
		
		b3Shape* shapeB = pair->shapeB;
		u32 proxyB = shapeB->m_broadPhaseID;
		b3Body* bodyB = shapeB->m_body;
		
		// Check if the bodies must not collide with each other.
		if (bodyA->ShouldCollide(bodyB) == false)
		{
			b3Contact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// Check for external filtering.
		if (m_contactFilter)
		{
			if (m_contactFilter->ShouldCollide(shapeA, shapeB) == false)
			{
				// The user has stopped the contact.
				b3Contact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}
		}

		// At least one body must be dynamic or kinematic.
		bool activeA = bodyA->IsAwake() && bodyA->m_type != e_staticBody;
		bool activeB = bodyB->IsAwake() && bodyB->m_type != e_staticBody;
		if (activeA == false && activeB == false) 
		{
			c = c->m_next;
			continue;
		}

		// Destroy the contact if the shape AABBs are not overlapping.
		bool overlap = m_broadPhase.TestOverlap(proxyA, proxyB);
		if (overlap == false)
		{
			b3Contact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update(m_contactListener);

		c = c->m_next;
	}
}

b3Contact* b3ContactManager::Create(b3Shape* shapeA, b3Shape* shapeB) 
{
	b3ShapeType typeA = shapeA->GetType();
	b3ShapeType typeB = shapeB->GetType();

	if (typeA > typeB) 
	{
		b3Swap(typeA, typeB);
		b3Swap(shapeA, shapeB);
	}

	B3_ASSERT(typeA <= typeB);

	b3Contact* c = NULL;
	if (typeA != e_meshShape && typeB != e_meshShape) 
	{
		void* block = m_convexBlocks.Allocate();
		b3ConvexContact* cxc = new (block) b3ConvexContact(shapeA, shapeB);
		c = cxc;
	}
	else 
	{
		if (typeB == e_meshShape) 
		{
			void* block = m_meshBlocks.Allocate();
			b3MeshContact* mxc = new (block) b3MeshContact(shapeA, shapeB);
			c = mxc;
		}
		else 
		{
			// Collisions between meshes are not implemented.
			return NULL;
		}
	}

	// The shapes might be swapped.
	c->m_pair.shapeA = shapeA;
	c->m_pair.shapeB = shapeB;
	return c;
}

void b3ContactManager::Destroy(b3Contact* c) 
{
	// Report to the contact listener the contact will be destroyed.
	if (m_contactListener) 
	{
		if (c->IsOverlapping()) 
		{
			m_contactListener->EndContact(c);
		}
	}
	
	b3OverlappingPair* pair = &c->m_pair;
	
	b3Shape* shapeA = c->GetShapeA();
	b3Shape* shapeB = c->GetShapeB();
	
	shapeA->m_contactEdges.Remove(&pair->edgeA);
	shapeB->m_contactEdges.Remove(&pair->edgeB);

	// Remove the contact from the world contact list.
	m_contactList.Remove(c);

	if (c->m_type == e_convexContact)
	{
		b3ConvexContact* cc = (b3ConvexContact*)c;
		cc->~b3ConvexContact();
		m_convexBlocks.Free(cc);
	}
	else
	{
		b3MeshContact* mc = (b3MeshContact*)c;
		
		// Remove the mesh contact from the world mesh contact list.
		m_meshContactList.Remove(&mc->m_link);
		
		mc->~b3MeshContact();
		m_meshBlocks.Free(mc);
	}
}