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

#include <bounce/cloth/cloth_contact_manager.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/shapes/cloth_sphere_shape.h>
#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/shapes/cloth_world_shape.h>
#include <bounce/cloth/cloth_particle.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3ClothContactManager::b3ClothContactManager() : 
	m_sphereAndTriangleContactBlocks(sizeof(b3ClothSphereAndTriangleContact)),
	m_sphereAndShapeContactBlocks(sizeof(b3ClothSphereAndShapeContact)),
	m_capsuleAndCapsuleContactBlocks(sizeof(b3ClothCapsuleAndCapsuleContact))
{

}

void b3ClothContactManager::FindNewContacts()
{
	B3_PROFILE("Cloth Find New Contacts");

	m_broadPhase.FindPairs(this);
}

void b3ClothContactManager::AddPair(void* data1, void* data2)
{
	b3ClothShape* shape1 = (b3ClothShape*)data1;
	b3ClothShape* shape2 = (b3ClothShape*)data2;

	if (shape1->m_type > shape2->m_type)
	{
		// Ensure type1 < type2.
		b3Swap(shape1, shape2);
	}

	if (shape1->m_type == e_clothSphereShape && shape2->m_type == e_clothSphereShape)
	{
		return;
	}

	if (shape1->m_type == e_clothSphereShape && shape2->m_type == e_clothCapsuleShape)
	{
		return;
	}

	if (shape1->m_type == e_clothCapsuleShape && shape2->m_type == e_clothTriangleShape)
	{
		return;
	}
	
	if (shape1->m_type == e_clothCapsuleShape && shape2->m_type == e_clothWorldShape)
	{
		return;
	}
	
	if (shape1->m_type == e_clothTriangleShape && shape2->m_type == e_clothTriangleShape)
	{
		return;
	}

	if (shape1->m_type == e_clothTriangleShape && shape2->m_type == e_clothWorldShape)
	{
		return;
	}
	
	if (shape1->m_type == e_clothWorldShape && shape2->m_type == e_clothWorldShape)
	{
		return;
	}
	
	if (shape1->m_type == e_clothSphereShape && shape2->m_type == e_clothWorldShape)
	{
		b3ClothSphereShape* s1 = (b3ClothSphereShape*)shape1;
		b3ClothParticle* p1 = s1->m_p;

		b3ClothWorldShape* ws2 = (b3ClothWorldShape*)shape2;
		const b3Shape* s2 = ws2->m_shape;
		const b3Body* b2 = s2->GetBody();

		if (b2->GetType() != e_staticBody)
		{
			// The cloth can't collide with non-static shapes.
			return;
		}

		if (p1->GetType() != e_dynamicClothParticle)
		{
			// The shapes must not collide with each other.
			return;
		}

		// Check if there is a contact between the two entities.
		for (b3ClothSphereAndShapeContact* c = m_sphereAndShapeContactList.m_head; c; c = c->m_next)
		{
			if (c->m_s1 == s1 && c->m_s2 == ws2)
			{
				// A contact already exists.
				return;
			}
		}

		// Create a new contact.
		b3ClothSphereAndShapeContact* c = CreateSphereAndShapeContact();

		c->m_s1 = s1;
		c->m_s2 = ws2;
		c->m_active = false;
		c->m_normalImpulse = scalar(0);
		c->m_tangentImpulse.SetZero();

		// Add the contact to the contact list.
		m_sphereAndShapeContactList.PushFront(c);

		return;
	}

	if (m_cloth->m_enableSelfCollision == true && 
		shape1->m_type == e_clothCapsuleShape && shape2->m_type == e_clothCapsuleShape)
	{
		b3ClothCapsuleShape* s1 = (b3ClothCapsuleShape*)shape1;
		b3ClothCapsuleShape* s2 = (b3ClothCapsuleShape*)shape2;

		b3ClothParticle* p1 = s1->m_p1;
		b3ClothParticle* p2 = s1->m_p2;

		b3ClothParticle* p3 = s2->m_p1;
		b3ClothParticle* p4 = s2->m_p2;

		bool isntDynamic1 = p1->m_type != e_dynamicClothParticle && p2->m_type != e_dynamicClothParticle;
		bool isntDynamic2 = p3->m_type != e_dynamicClothParticle && p4->m_type != e_dynamicClothParticle;

		if (isntDynamic1 && isntDynamic2)
		{
			// The entities must not collide with each other.
			return;
		}

		// Do the edges share a vertex?
		if (p1 == p3 || p1 == p4)
		{
			return;
		}

		if (p2 == p3 || p2 == p4)
		{
			return;
		}

		// Check if there is a contact between the two capsules.
		for (b3ClothCapsuleAndCapsuleContact* c = m_capsuleAndCapsuleContactList.m_head; c; c = c->m_next)
		{
			if (c->m_s1 == s1 && c->m_s2 == s2)
			{
				// A contact already exists.
				return;
			}
			
			if (c->m_s1 == s2 && c->m_s2 == s1)
			{
				// A contact already exists.
				return;
			}
		}

		// Create a new contact.
		b3ClothCapsuleAndCapsuleContact* c = CreateCapsuleAndCapsuleContact();

		c->m_s1 = s1;
		c->m_s2 = s2;
		c->m_normalImpulse = scalar(0);
		c->m_tangentImpulse.SetZero();
		c->m_active = false;

		// Add the contact to the cloth contact list.
		m_capsuleAndCapsuleContactList.PushFront(c);
		
		return;
	}

	if (m_cloth->m_enableSelfCollision == true && 
		shape1->m_type == e_clothSphereShape && shape2->m_type == e_clothTriangleShape)
	{
		b3ClothSphereShape* s1 = (b3ClothSphereShape*)shape1;
		b3ClothTriangleShape* s2 = (b3ClothTriangleShape*)shape2;

		b3ClothParticle* p1 = s1->m_p;

		b3ClothParticle* p2 = s2->m_p1;
		b3ClothParticle* p3 = s2->m_p2;
		b3ClothParticle* p4 = s2->m_p3;

		bool isntDynamic1 = p1->m_type != e_dynamicClothParticle;
		bool isntDynamic2 = p2->m_type != e_dynamicClothParticle && p3->m_type != e_dynamicClothParticle && p4->m_type != e_dynamicClothParticle;

		if (isntDynamic1 && isntDynamic2)
		{
			// The entities must not collide with each other.
			return;
		}

		if (p1 == p2 || p1 == p3 || p1 == p4)
		{
			// The entities must not collide with each other.
			return;
		}

		// Check if there is a contact between the two entities.
		for (b3ClothSphereAndTriangleContact* c = m_sphereAndTriangleContactList.m_head; c; c = c->m_next)
		{
			if (c->m_s1 == s1 && c->m_s2 == s2)
			{
				// A contact already exists.
				return;
			}
		}

		// Create a new contact.
		b3ClothSphereAndTriangleContact* c = CreateSphereAndTriangleContact();

		c->m_s1 = s1;
		c->m_s2 = s2;
		c->m_normalImpulse = scalar(0);
		c->m_tangentImpulse.SetZero();
		c->m_active = false;

		// Add the contact to the cloth contact list.
		m_sphereAndTriangleContactList.PushFront(c);

		return;
	}
}

b3ClothSphereAndTriangleContact* b3ClothContactManager::CreateSphereAndTriangleContact()
{
	void* block = m_sphereAndTriangleContactBlocks.Allocate();
	return new(block) b3ClothSphereAndTriangleContact();
}

void b3ClothContactManager::Destroy(b3ClothSphereAndTriangleContact* c)
{
	m_sphereAndTriangleContactList.Remove(c);
	c->~b3ClothSphereAndTriangleContact();
	m_sphereAndTriangleContactBlocks.Free(c);
}

b3ClothSphereAndShapeContact* b3ClothContactManager::CreateSphereAndShapeContact()
{
	void* block = m_sphereAndShapeContactBlocks.Allocate();
	return new(block) b3ClothSphereAndShapeContact();
}

void b3ClothContactManager::Destroy(b3ClothSphereAndShapeContact* c)
{
	m_sphereAndShapeContactList.Remove(c);
	c->~b3ClothSphereAndShapeContact();
	m_sphereAndShapeContactBlocks.Free(c);
}

b3ClothCapsuleAndCapsuleContact* b3ClothContactManager::CreateCapsuleAndCapsuleContact()
{
	void* block = m_capsuleAndCapsuleContactBlocks.Allocate();
	return new(block) b3ClothCapsuleAndCapsuleContact();
}

void b3ClothContactManager::Destroy(b3ClothCapsuleAndCapsuleContact* c)
{
	m_capsuleAndCapsuleContactList.Remove(c);
	c->~b3ClothCapsuleAndCapsuleContact();
	m_capsuleAndCapsuleContactBlocks.Free(c);
}

void b3ClothContactManager::UpdateContacts()
{
	B3_PROFILE("Cloth Update Contacts");
	
	{
		// Update the state of sphere and shape contacts.
		b3ClothSphereAndShapeContact* c = m_sphereAndShapeContactList.m_head;
		while (c)
		{
			b3ClothSphereShape* s1 = c->m_s1;
			b3ClothParticle* p1 = s1->m_p;

			b3ClothWorldShape* ws2 = c->m_s2;
			const b3Shape* s2 = ws2->m_shape;
			const b3Body* b2 = s2->GetBody();

			// Cease the contact if body became non-static.
			if (b2->GetType() != e_staticBody)
			{
				b3ClothSphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// Cease the contact if entities must not collide with each other.
			if (p1->m_type != e_dynamicClothParticle)
			{
				b3ClothSphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_broadPhaseId;
			u32 proxy2 = c->m_s2->m_broadPhaseId;

			// Destroy the contact if AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3ClothSphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}

	{
		// Update the state of sphere and triangle contacts.
		b3ClothSphereAndTriangleContact* c = m_sphereAndTriangleContactList.m_head;
		while (c)
		{
			b3ClothSphereShape* s1 = c->m_s1;
			b3ClothParticle* p1 = s1->m_p;

			b3ClothTriangleShape* s2 = c->m_s2;
			b3ClothParticle* p2 = s2->m_p1;
			b3ClothParticle* p3 = s2->m_p2;
			b3ClothParticle* p4 = s2->m_p3;

			bool isntDynamic1 = p1->m_type != e_dynamicClothParticle;
			
			bool isntDynamic2 = 
				p2->m_type != e_dynamicClothParticle && 
				p3->m_type != e_dynamicClothParticle && 
				p4->m_type != e_dynamicClothParticle;

			// Destroy the contact if shapes must not collide with each other.
			if (isntDynamic1 && isntDynamic2)
			{
				b3ClothSphereAndTriangleContact* quack = c;
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
				b3ClothSphereAndTriangleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}

	{
		// Update the state of capsule contacts.
		b3ClothCapsuleAndCapsuleContact* c = m_capsuleAndCapsuleContactList.m_head;
		while (c)
		{
			b3ClothCapsuleShape* s1 = c->m_s1;
			b3ClothParticle* p1 = s1->m_p1;
			b3ClothParticle* p2 = s1->m_p2;

			b3ClothCapsuleShape* s2 = c->m_s2;
			b3ClothParticle* p3 = s2->m_p1;
			b3ClothParticle* p4 = s2->m_p2;
			
			bool isntDynamic1 = p1->m_type != e_dynamicClothParticle && p2->m_type != e_dynamicClothParticle;
			bool isntDynamic2 = p3->m_type != e_dynamicClothParticle && p4->m_type != e_dynamicClothParticle;

			// Destroy the contact if primitives must not collide with each other.
			if (isntDynamic1 && isntDynamic2)
			{
				b3ClothCapsuleAndCapsuleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_broadPhaseId;
			u32 proxy2 = c->m_s2->m_broadPhaseId;

			// Destroy the contact if AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3ClothCapsuleAndCapsuleContact* quack = c;
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