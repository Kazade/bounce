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

#ifndef B3_CLOTH_CAPSULE_AND_CAPSULE_CONTACT_H
#define B3_CLOTH_CAPSULE_AND_CAPSULE_CONTACT_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>

class b3ClothCapsuleShape;

// Contact between capsules
class b3ClothCapsuleAndCapsuleContact
{
public:
private:
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ClothCapsuleShape;
	friend class b3ClothContactManager;
	friend class b3ClothContactSolver;
	friend class b3List2<b3ClothCapsuleAndCapsuleContact>;

	b3ClothCapsuleAndCapsuleContact() { }
	~b3ClothCapsuleAndCapsuleContact() { }

	void Update();

	b3ClothCapsuleShape* m_s1;
	b3ClothCapsuleShape* m_s2;

	bool m_active;

	scalar m_w1, m_w2, m_w3, m_w4;

	b3Vec3 m_normal1;
	scalar m_normalImpulse;

	b3Vec3 m_tangent1;
	b3Vec3 m_tangent2;
	b3Vec2 m_tangentImpulse;

	b3ClothCapsuleAndCapsuleContact* m_prev;
	b3ClothCapsuleAndCapsuleContact* m_next;
};

#endif