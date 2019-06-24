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

#include <bounce/cloth/contacts/cloth_particle_body_contact.h>
#include <bounce/cloth/contacts/cloth_particle_triangle_contact.h>
#include <bounce/collision/broad_phase.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/list.h>

class b3Cloth;

// Contact delegator for b3Cloth.
class b3ClothContactManager
{
public:
	b3ClothContactManager();

	void FindNewContacts();
	
	void AddPair(void* data1, void* data2);
	void FindNewClothContacts();

	void AddPSPair(b3Particle* p1, b3Shape* s2);
	void FindNewBodyContacts();

	void UpdateContacts();
	void UpdateClothContacts();
	void UpdateBodyContacts();

	b3ParticleTriangleContact* CreateParticleTriangleContact();
	void Destroy(b3ParticleTriangleContact* c);

	b3ParticleBodyContact* CreateParticleBodyContact();
	void Destroy(b3ParticleBodyContact* c);

	b3BlockPool m_particleTriangleContactBlocks;
	b3BlockPool m_particleBodyContactBlocks;

	b3Cloth* m_cloth;
	b3BroadPhase m_broadPhase;
	b3List2<b3ParticleTriangleContact> m_particleTriangleContactList;
	b3List2<b3ParticleBodyContact> m_particleBodyContactList;
};

#endif