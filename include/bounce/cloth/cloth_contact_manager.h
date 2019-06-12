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

#ifndef B3_CLOTH_CONTACT_MANAGER_H
#define B3_CLOTH_CONTACT_MANAGER_H

#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/list.h>
#include <bounce/collision/broad_phase.h>

class b3Cloth;
class b3Particle;
struct b3ClothMeshTriangle;
struct b3ClothAABBProxy;

// Contact between particle and a triangle
class b3ParticleTriangleContact
{
public:
	b3Particle* m_p1;
	
	b3ClothMeshTriangle* m_triangle;
	b3ClothAABBProxy* m_triangleProxy;
	b3Particle* m_p2;
	b3Particle* m_p3;
	b3Particle* m_p4;

	bool m_front;
	
	bool m_active;

	float32 m_normalImpulse;

	b3ParticleTriangleContact* m_prev;
	b3ParticleTriangleContact* m_next;
};

// Contact delegator for b3Cloth.
class b3ClothContactManager
{
public:
	b3ClothContactManager();

	// The broad-phase callback.
	void AddPair(void* data1, void* data2);

	void FindNewContacts();

	void UpdateContacts();
	
	b3ParticleTriangleContact* Create();
	
	void Destroy(b3ParticleTriangleContact* c);

	void Update(b3ParticleTriangleContact* c);

	b3Cloth* m_cloth;

	b3BlockPool m_particleTriangleContactBlocks;
	
	b3BroadPhase m_broadPhase;

	b3List2<b3ParticleTriangleContact> m_particleTriangleContactList;
};

#endif