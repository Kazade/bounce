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

b3ClothContactManager::b3ClothContactManager() : 
	m_particleTriangleContactBlocks(sizeof(b3ParticleTriangleContact))
{

}

void b3ClothContactManager::FindNewContacts()
{
	B3_PROFILE("Cloth Find New Contacts");
	
	m_broadPhase.FindPairs(this);
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

	b3Particle* p1 = (b3Particle*)proxy1->data;

	b3ClothMeshTriangle* triangle = (b3ClothMeshTriangle*)proxy2->data;
	b3Particle* p2 = m_cloth->m_vertexParticles[triangle->v1];
	b3Particle* p3 = m_cloth->m_vertexParticles[triangle->v2];
	b3Particle* p4 = m_cloth->m_vertexParticles[triangle->v3];

	// Check if there is a contact between the two entities.
	for (b3ParticleTriangleContact* c = m_particleTriangleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == p1 && c->m_triangle == triangle)
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

	if (triangle->v1 == p1->m_vertex || triangle->v2 == p1->m_vertex || triangle->v3 == p1->m_vertex)
	{
		// The entities must not collide with each other.
		return;
	}
	
	// Create a new contact.
	b3ParticleTriangleContact* c = CreateParticleTriangleContact();

	c->m_p1 = p1;
	c->m_triangle = triangle;
	c->m_triangleProxy = proxy2;
	c->m_p2 = p2;
	c->m_p3 = p3;
	c->m_p4 = p4;
	c->m_normalImpulse = 0.0f;
	c->m_front = false;
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

void b3ClothContactManager::UpdateContacts()
{
	B3_PROFILE("Cloth Update Contacts");
	
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

		u32 proxy1 = c->m_p1->m_aabbProxy.broadPhaseId;
		u32 proxy2 = c->m_triangleProxy->broadPhaseId;

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