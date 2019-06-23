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
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_listeners.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

b3ClothContactManager::b3ClothContactManager() : 
	m_particleTriangleContactBlocks(sizeof(b3ParticleTriangleContact)),
	m_particleBodyContactBlocks(sizeof(b3ParticleBodyContact))
{

}

void b3ClothContactManager::FindNewContacts()
{
	FindNewClothContacts();
	FindNewBodyContacts();
}

void b3ClothContactManager::FindNewClothContacts()
{
	B3_PROFILE("Cloth Find New Cloth Contacts");
	
	m_broadPhase.FindPairs(this);
}

class b3ClothContactManagerFindNewBodyContactsQueryListener : public b3QueryListener
{
public:
	virtual bool ReportShape(b3Shape* s2)
	{
		cm->AddPSPair(p1, s2);
		
		// Keep looking for overlaps
		return true;
	}
	
	b3ClothContactManager* cm;
	b3Particle* p1;
};

void b3ClothContactManager::FindNewBodyContacts()
{
	B3_PROFILE("Cloth Find New Body Contacts");

	// Is there a world attached to this cloth?
	if (m_cloth->m_world == nullptr)
	{
		return;
	}

	for (b3Particle* p = m_cloth->m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_type != e_dynamicParticle)
		{
			continue;
		}

		b3AABB3 aabb = m_broadPhase.GetAABB(p->m_broadPhaseId);

		b3ClothContactManagerFindNewBodyContactsQueryListener listener;
		listener.cm = this;
		listener.p1 = p;

		m_cloth->m_world->QueryAABB(&listener, aabb);
	}
}

void b3ClothContactManager::AddPSPair(b3Particle* p1, b3Shape* s2)
{
	// Check if there is a contact between the two entities.
	for (b3ParticleBodyContact* c = m_particleBodyContactList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == p1 && c->m_s2 == s2)
		{
			// A contact already exists.
			return;
		}
	}

	bool isntDynamic1 = p1->m_type != e_dynamicParticle;
	bool isntDynamic2 = s2->GetBody()->GetType() != e_dynamicBody;

	if (isntDynamic1 && isntDynamic2)
	{
		// The entities must not collide with each other.
		return;
	}

	// Create a new contact.
	b3ParticleBodyContact* c = CreateParticleBodyContact();

	c->m_p1 = p1;
	c->m_s2 = s2;
	c->m_active = false;
	c->m_normalImpulse = 0.0f;
	c->m_tangentImpulse.SetZero();

	// Add the contact to the body contact list.
	m_particleBodyContactList.PushFront(c);
}

void b3ClothContactManager::AddPair(void* data1, void* data2)
{
	b3ClothAABBProxy* proxy1 = (b3ClothAABBProxy*)data1;
	b3ClothAABBProxy* proxy2 = (b3ClothAABBProxy*)data2;

	if (proxy1->type == e_particleProxy && proxy2->type == e_particleProxy)
	{
		// Particle-particle contacts are not supported.
		return;
	}

	if (proxy1->type == e_triangleProxy && proxy2->type == e_triangleProxy)
	{
		// Triangle-triangle contacts are not supported.
		return;
	}

	if (proxy1->type == e_triangleProxy)
	{
		// Ensure proxy1 is a particle and proxy 2 a triangle.
		b3Swap(proxy1, proxy2);
	}

	B3_ASSERT(proxy1->type == e_particleProxy);
	B3_ASSERT(proxy2->type == e_triangleProxy);

	b3Particle* p1 = (b3Particle*)proxy1->owner;

	b3ClothTriangle* t2 = (b3ClothTriangle*)proxy2->owner;
	b3ClothMeshTriangle* triangle = m_cloth->m_mesh->triangles + t2->m_triangle;
	b3Particle* p2 = m_cloth->m_particles[triangle->v1];
	b3Particle* p3 = m_cloth->m_particles[triangle->v2];
	b3Particle* p4 = m_cloth->m_particles[triangle->v3];

	// Check if there is a contact between the two entities.
	for (b3ParticleTriangleContact* c = m_particleTriangleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == p1 && c->m_t2 == t2)
		{
			// A contact already exists.
			return;
		}
	}

	bool isntDynamic1 = p1->m_type != e_dynamicParticle;
	bool isntDynamic2 = p2->m_type != e_dynamicParticle && p3->m_type != e_dynamicParticle && p4->m_type != e_dynamicParticle;

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
	
	// Create a new contact.
	b3ParticleTriangleContact* c = CreateParticleTriangleContact();

	c->m_p1 = p1;
	c->m_t2 = t2;
	c->m_p2 = p2;
	c->m_p3 = p3;
	c->m_p4 = p4;
	c->m_normalImpulse = 0.0f;
	c->m_tangentImpulse1 = 0.0f;
	c->m_tangentImpulse2 = 0.0f;
	c->m_active = false;

	// Add the contact to the cloth contact list.
	m_particleTriangleContactList.PushFront(c);
}

b3ParticleTriangleContact* b3ClothContactManager::CreateParticleTriangleContact()
{
	void* block = m_particleTriangleContactBlocks.Allocate();
	return new(block) b3ParticleTriangleContact();
}

void b3ClothContactManager::Destroy(b3ParticleTriangleContact* c)
{
	m_particleTriangleContactList.Remove(c);

	c->~b3ParticleTriangleContact();
	
	m_particleTriangleContactBlocks.Free(c);
}

b3ParticleBodyContact* b3ClothContactManager::CreateParticleBodyContact()
{
	void* block = m_particleBodyContactBlocks.Allocate();
	return new(block) b3ParticleBodyContact();
}

void b3ClothContactManager::Destroy(b3ParticleBodyContact* c)
{
	m_particleBodyContactList.Remove(c);

	c->~b3ParticleBodyContact();

	m_particleBodyContactBlocks.Free(c);
}

void b3ClothContactManager::UpdateContacts()
{
	UpdateClothContacts();
	UpdateBodyContacts();
}

void b3ClothContactManager::UpdateClothContacts()
{
	B3_PROFILE("Cloth Update Cloth Contacts");
	
	// Update the state of particle-triangle contacts.
	b3ParticleTriangleContact* c = m_particleTriangleContactList.m_head;
	while (c)
	{
		bool isntDynamic1 = c->m_p1->m_type != e_dynamicParticle;
		bool isntDynamic2 = c->m_p2->m_type != e_dynamicParticle && c->m_p3->m_type != e_dynamicParticle && c->m_p4->m_type != e_dynamicParticle;

		// Destroy the contact if primitives must not collide with each other.
		if (isntDynamic1 && isntDynamic2)
		{
			b3ParticleTriangleContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		u32 proxy1 = c->m_p1->m_broadPhaseId;
		u32 proxy2 = c->m_t2->m_broadPhaseId;

		// Destroy the contact if primitive AABBs are not overlapping.
		bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
		if (overlap == false)
		{
			b3ParticleTriangleContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update();

		c = c->m_next;
	}
}

void b3ClothContactManager::UpdateBodyContacts()
{
	B3_PROFILE("Cloth Update Body Contacts");
	
	// Update the state of particle-body contacts.
	b3ParticleBodyContact* c = m_particleBodyContactList.m_head;
	while (c)
	{
		bool isntDynamic1 = c->m_p1->m_type != e_dynamicParticle;
		bool isntDynamic2 = c->m_s2->GetType() != e_dynamicBody;

		// Cease the contact if entities must not collide with each other.
		if (isntDynamic1 && isntDynamic2)
		{
			b3ParticleBodyContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		b3AABB3 aabb1 = m_broadPhase.GetAABB(c->m_p1->m_broadPhaseId);
		b3AABB3 aabb2 = c->m_s2->GetAABB();

		// Destroy the contact if entities AABBs are not overlapping.
		bool overlap = b3TestOverlap(aabb1, aabb2);
		if (overlap == false)
		{
			b3ParticleBodyContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update();

		c = c->m_next;
	}
}