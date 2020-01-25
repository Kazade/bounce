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

#ifndef B3_CLOTH_SPHERE_AND_SHAPE_CONTACT_H
#define B3_CLOTH_SPHERE_AND_SHAPE_CONTACT_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>
#include <bounce/common/math/transform.h>

class b3ClothSphereShape;
class b3ClothWorldShape;

// A contact between a sphere and a shape
class b3ClothSphereAndShapeContact
{
public:
private:
	friend class b3List2<b3ClothSphereAndShapeContact>;
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ClothSphereShape;
	friend class b3ClothWorldShape;
	friend class b3ClothContactManager;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;
	friend struct b3ClothSphereAndShapeContactWorldPoint;

	b3ClothSphereAndShapeContact() { }
	~b3ClothSphereAndShapeContact() { }

	void Update();

	b3ClothSphereShape* m_s1;
	b3ClothWorldShape* m_s2;

	bool m_active;
	
	b3Vec3 m_normal2;
	b3Vec3 m_point2;
	scalar m_normalImpulse;

	b3Vec3 m_tangent1, m_tangent2;
	b3Vec2 m_tangentImpulse;

	b3ClothSphereAndShapeContact* m_prev;
	b3ClothSphereAndShapeContact* m_next;
};

struct b3ClothSphereAndShapeContactWorldPoint
{
	void Initialize(const b3ClothSphereAndShapeContact* c, scalar rA, const b3Vec3& cA, scalar rB);

	b3Vec3 point;
	b3Vec3 normal;
	scalar separation;
};

#endif