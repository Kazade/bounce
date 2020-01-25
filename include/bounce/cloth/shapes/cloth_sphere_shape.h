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

#ifndef B3_CLOTH_SPHERE_SHAPE_H
#define B3_CLOTH_SPHERE_SHAPE_H

#include <bounce/cloth/shapes/cloth_shape.h>
#include <bounce/collision/shapes/aabb.h>
#include <bounce/common/template/list.h>

class b3ClothParticle;

struct b3ClothSphereShapeDef : b3ClothShapeDef
{
	b3ClothParticle* p;
};

// A cloth sphere shape
class b3ClothSphereShape : public b3ClothShape
{
public:
	// Return the cloth particle.
	b3ClothParticle* GetParticle() { return m_p; }
	const b3ClothParticle* GetParticle() const { return m_p; }

	// Return the next sphere in the cloth.
	b3ClothSphereShape* GetNext() { return m_next; };
	const b3ClothSphereShape* GetNext() const { return m_next; };
private:
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ClothContactManager;
	friend class b3ClothSphereAndTriangleContact;
	friend class b3ClothSphereAndShapeContact;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;
	friend class b3List2<b3ClothSphereShape>;

	b3ClothSphereShape(const b3ClothSphereShapeDef& def, b3Cloth* cloth);
	~b3ClothSphereShape();

	// Compute AABB
	b3AABB ComputeAABB() const;

	// Synchronize AABB
	void Synchronize(const b3Vec3& displacement);

	// Destroy contacts
	void DestroyContacts();

	// Particle
	b3ClothParticle* m_p;

	// Links to the cloth sphere shape list.
	b3ClothSphereShape* m_prev;
	b3ClothSphereShape* m_next;
};

#endif