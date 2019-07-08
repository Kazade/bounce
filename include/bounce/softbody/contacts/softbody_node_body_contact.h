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

#ifndef B3_SOFT_BODY_NODE_BODY_CONTACT_H
#define B3_SOFT_BODY_NODE_BODY_CONTACT_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>
#include <bounce/common/math/transform.h>

struct b3SoftBodyNode;
class b3Shape;

// A contact between a node and a body
class b3NodeBodyContact
{
public:
private:
	friend class b3List2<b3NodeBodyContact>;
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend struct b3SoftBodyNode;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyContactSolver;
	friend struct b3NodeBodyContactWorldPoint;

	b3NodeBodyContact() { }
	~b3NodeBodyContact() { }
	
	void Update();

	b3SoftBodyNode* m_n1;
	b3Shape* m_s2;

	// Is the contact active?
	bool m_active;
	
	// Contact constraint
	b3Vec3 m_normal1;
	b3Vec3 m_localPoint1;
	b3Vec3 m_localPoint2;
	float32 m_normalImpulse;

	// Friction constraint
	b3Vec3 m_tangent1, m_tangent2;
	b3Vec2 m_tangentImpulse;

	// List pointers into the soft body
	b3NodeBodyContact* m_prev;
	b3NodeBodyContact* m_next;
};

struct b3NodeBodyContactWorldPoint
{
	void Initialize(const b3NodeBodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB);

	b3Vec3 point;
	b3Vec3 normal;
	float32 separation;
};

#endif 