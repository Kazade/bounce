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

#ifndef B3_CLOTH_CAPSULE_SHAPE_H
#define B3_CLOTH_CAPSULE_SHAPE_H

#include <bounce/cloth/shapes/cloth_shape.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3ClothParticle;

struct b3ClothCapsuleShapeDef : b3ClothShapeDef
{
	b3ClothParticle* p1;
	b3ClothParticle* p2;
};

// A cloth capsule shape
class b3ClothCapsuleShape : public b3ClothShape
{
public:
	// Return the particle 1.
	b3ClothParticle* GetParticle1() { return m_p1; }
	const b3ClothParticle* GetParticle1() const { return m_p1; }

	// Return the particle 2.
	b3ClothParticle* GetParticle2() { return m_p2; }
	const b3ClothParticle* GetParticle2() const { return m_p2; }

	// Return the next capsule in the cloth capsule list.
	b3ClothCapsuleShape* GetNext() { return m_next; }
	const b3ClothCapsuleShape* GetNext() const { return m_next; }
private:
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ShearForce;
	friend class b3StretchForce;
	friend class b3MouseForce;
	friend class b3ClothContactManager;
	friend class b3ClothSphereAndTriangleContact;
	friend class b3ClothCapsuleAndCapsuleContact;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;
	friend class b3List2<b3ClothCapsuleShape>;

	b3ClothCapsuleShape(const b3ClothCapsuleShapeDef& def, b3Cloth* cloth);
	~b3ClothCapsuleShape();

	// Compute AABB
	b3AABB ComputeAABB() const;
	
	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Destroy contacts
	void DestroyContacts();

	// Particles
	b3ClothParticle* m_p1;
	b3ClothParticle* m_p2;

	// Links to the cloth capsule shape list.
	b3ClothCapsuleShape* m_prev;
	b3ClothCapsuleShape* m_next;
};

#endif