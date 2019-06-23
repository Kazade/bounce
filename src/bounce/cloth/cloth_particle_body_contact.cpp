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

#include <bounce/cloth/cloth_particle_body_contact.h>
#include <bounce/cloth/particle.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>

void b3ParticleBodyContact::Update()
{
	b3Sphere sphere;
	sphere.radius = m_p1->m_radius;
	sphere.vertex = m_p1->m_position;

	b3Shape* shape = m_s2;
	b3Body* body = shape->GetBody();
	b3Transform xf = body->GetTransform();

	b3TestSphereOutput out;
	if (shape->TestSphere(&out, sphere, xf) == false)
	{
		m_active = false;
		return;
	}

	m_active = true;
	m_normal1 = -out.normal;
	m_localPoint1.SetZero();
	m_localPoint2 = body->GetLocalPoint(out.point);
	m_tangent1 = b3Perp(m_normal1);
	m_tangent2 = b3Cross(m_tangent1, m_normal1);
}

void b3ParticleBodyContactWorldPoint::Initialize(const b3ParticleBodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB)
{
	b3Vec3 nA = c->m_normal1;

	b3Vec3 cA = xfA * c->m_localPoint1;
	b3Vec3 cB = xfB * c->m_localPoint2;

	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = 0.5f * (pA + pB);
	normal = nA;
	separation = b3Dot(cB - cA, nA) - rA - rB;
}
