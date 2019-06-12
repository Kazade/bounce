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

#include <bounce/common/draw.h>

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
	b3ParticleTriangleContact* c = Create();

	c->m_p1 = p1;
	
	c->m_p2 = p2;
	c->m_p3 = p3;
	c->m_p4 = p4;
	c->m_triangle = triangle;
	c->m_triangleProxy = proxy2;

	c->m_front = false;
	c->m_active = false;

	c->m_normalImpulse = 0.0f;

	// Add the contact to the cloth contact list.
	m_particleTriangleContactList.PushFront(c);
}

b3ParticleTriangleContact* b3ClothContactManager::Create()
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

static void b3Solve3(float32 out[3],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	// Test vertex regions
	float32 wAB[3], wBC[3], wCA[3];
	b3BarycentricCoordinates(wAB, A, B, Q);
	b3BarycentricCoordinates(wBC, B, C, Q);
	b3BarycentricCoordinates(wCA, C, A, Q);

	// R A
	if (wAB[1] <= 0.0f && wCA[0] <= 0.0f)
	{
		out[0] = 1.0f;
		out[1] = 0.0f;
		out[2] = 0.0f;
		return;
	}

	// R B
	if (wAB[0] <= 0.0f && wBC[1] <= 0.0f)
	{
		out[0] = 0.0f;
		out[1] = 1.0f;
		out[2] = 0.0f;
		return;
	}

	// R C
	if (wBC[0] <= 0.0f && wCA[1] <= 0.0f)
	{
		out[0] = 0.0f;
		out[1] = 0.0f;
		out[2] = 1.0f;
		return;
	}

	// Test edge regions		
	float32 wABC[4];
	b3BarycentricCoordinates(wABC, A, B, C, Q);

	// R AB
	if (wAB[0] > 0.0f && wAB[1] > 0.0f && wABC[3] * wABC[2] <= 0.0f)
	{
		float32 divisor = wAB[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = s * wAB[0];
		out[1] = s * wAB[1];
		out[2] = 0.0f;
		return;
	}

	// R BC
	if (wBC[0] > 0.0f && wBC[1] > 0.0f && wABC[3] * wABC[0] <= 0.0f)
	{
		float32 divisor = wBC[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = 0.0f;
		out[1] = s * wBC[0];
		out[2] = s * wBC[1];
		return;
	}

	// R CA
	if (wCA[0] > 0.0f && wCA[1] > 0.0f && wABC[3] * wABC[1] <= 0.0f)
	{
		float32 divisor = wCA[2];
		B3_ASSERT(divisor > 0.0f);
		float32 s = 1.0f / divisor;
		out[0] = s * wCA[1];
		out[1] = 0.0f;
		out[2] = s * wCA[0];
		return;
	}

	// R ABC/ACB
	float32 divisor = wABC[3];
	if (divisor == 0.0f)
	{
		float32 s = 1.0f / 3.0f;
		out[0] = s;
		out[1] = s;
		out[2] = s;
		return;
	}

	B3_ASSERT(divisor > 0.0f);
	float32 s = 1.0f / divisor;
	out[0] = s * wABC[0];
	out[1] = s * wABC[1];
	out[2] = s * wABC[2];
}

void b3ClothContactManager::Update(b3ParticleTriangleContact* c)
{
	b3Particle* p1 = c->m_p1;
	
	b3Particle* p2 = c->m_p2;
	b3Particle* p3 = c->m_p3;
	b3Particle* p4 = c->m_p4;

	float32 r1 = p1->m_radius;
	float32 r2 = 0.0f;

	float32 totalRadius = r1 + r2;

	b3Vec3 A = p2->m_position;
	b3Vec3 B = p3->m_position;
	b3Vec3 C = p4->m_position;

	b3Vec3 n = b3Cross(B - A, C - A);
	float32 len = n.Normalize();

	// Is ABC degenerate?
	if (len == 0.0f)
	{
		c->m_active = false;
		return;
	}

	b3Vec3 P1 = p1->m_position;

	float32 distance = b3Dot(n, P1 - A);

	// Is P1 below the plane?
	if (distance < -totalRadius)
	{
		c->m_active = false;
		return;
	}

	// Is P1 above the plane?
	if (distance > totalRadius)
	{
		c->m_active = false;
		return;
	}

	// Closest point on ABC to P1
	float32 wABC[3];
	b3Solve3(wABC, A, B, C, P1);

	b3Vec3 P2 = wABC[0] * A + wABC[1] * B + wABC[2] * C;

	if (b3DistanceSquared(P1, P2) > totalRadius * totalRadius)
	{
		c->m_active = false;
		return;
	}

	// Activate the contact
	c->m_active = true;

	// Is the the other point in front of the plane? 
	if (distance >= 0.0f)
	{
		c->m_front = true;
	}
	else
	{
		c->m_front = false;
	}
}

void b3ClothContactManager::UpdateContacts()
{
	B3_PROFILE("Cloth Update Contacts");
	
	// Update the state of all triangle contacts.
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
		Update(c);

		c = c->m_next;
	}
}