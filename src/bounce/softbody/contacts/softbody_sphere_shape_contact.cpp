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

#include <bounce/softbody/contacts/softbody_sphere_shape_contact.h>
#include <bounce/softbody/shapes/softbody_sphere_shape.h>
#include <bounce/softbody/shapes/softbody_world_shape.h>
#include <bounce/softbody/softbody_node.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

void b3SoftBodySphereAndShapeContact::Update()
{
	b3Sphere sphere;
	sphere.radius = m_s1->m_radius;
	sphere.vertex = m_s1->m_node->m_position;

	const b3Shape* shape = m_s2->m_shape;
	const b3Body* body = shape->GetBody();
	b3Transform xf = body->GetTransform();

	b3TestSphereOutput out;
	if (shape->TestSphere(&out, sphere, xf) == false)
	{
		m_active = false;
		return;
	}

	m_active = true;
	m_normal2 = out.normal;
	m_point2 = out.point;
	m_tangent1 = b3Perp(m_normal2);
	m_tangent2 = b3Cross(m_tangent1, m_normal2);
}

void b3SoftBodySphereAndShapeContactWorldPoint::Initialize(const b3SoftBodySphereAndShapeContact* c, scalar rA, const b3Vec3& cA, scalar rB)
{
	b3Vec3 cB = c->m_point2;
	b3Vec3 nB = c->m_normal2;

	b3Vec3 pA = cA - rA * nB;
	b3Vec3 pB = cB + rB * nB;

	point = scalar(0.5) * (pA + pB);
	normal = nB;
	separation = b3Dot(cA - cB, nB) - rA - rB;
}
