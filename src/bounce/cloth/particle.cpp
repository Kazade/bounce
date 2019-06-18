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

#include <bounce/cloth/particle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/cloth_triangle.h>

void b3ParticleBodyContactWorldPoint::Initialize(const b3ParticleBodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB)
{
	b3Vec3 nA = c->normal1;

	b3Vec3 cA = b3Mul(xfA, c->localPoint1);
	b3Vec3 cB = b3Mul(xfB, c->localPoint2);

	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = 0.5f * (pA + pB);
	normal = nA;
	separation = b3Dot(cB - cA, nA) - rA - rB;
}

b3Particle::b3Particle(const b3ParticleDef& def, b3Cloth* cloth)
{
	m_cloth = cloth;
	m_type = def.type;
	m_position = def.position;
	m_velocity = def.velocity;
	m_force = def.force;
	m_translation.SetZero();
	m_mass = def.mass;

	if (m_mass == 0.0f)
	{
		m_type = e_staticParticle;
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}
	else
	{
		m_type = e_dynamicParticle;
		m_invMass = 1.0f / m_mass;
	}

	m_radius = def.radius;
	m_friction = def.friction;
	m_userData = nullptr;
	m_x.SetZero();
	m_vertex = ~0;
	m_bodyContact.active = false;
}

b3Particle::~b3Particle()
{

}

void b3Particle::Synchronize(const b3Vec3& displacement)
{
	b3AABB3 aabb;
	aabb.Set(m_position, m_radius);

	m_cloth->m_contactManager.m_broadPhase.MoveProxy(m_aabbProxy.broadPhaseId, aabb, displacement);
}

void b3Particle::SynchronizeTriangles()
{
	if (m_vertex == ~0)
	{
		return;
	}

	for (u32 i = 0; i < m_cloth->m_mesh->triangleCount; ++i)
	{
		b3ClothMeshTriangle* triangle = m_cloth->m_mesh->triangles + i;

		if (triangle->v1 == m_vertex || triangle->v2 == m_vertex || triangle->v3 == m_vertex)
		{
			m_cloth->GetTriangle(i)->Synchronize(b3Vec3_zero);
		}
	}
}

void b3Particle::DestroyContacts()
{
	// Destroy body contacts
	m_bodyContact.active = false;

	// Destroy triangle contacts
	b3ParticleTriangleContact* c = m_cloth->m_contactManager.m_particleTriangleContactList.m_head;
	while (c)
	{
		if (c->m_p1 == this)
		{
			b3ParticleTriangleContact* quack = c;
			c = c->m_next;
			m_cloth->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}

void b3Particle::SetType(b3ParticleType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;
	m_force.SetZero();

	if (type == e_staticParticle)
	{
		m_velocity.SetZero();
		m_translation.SetZero();
		
		Synchronize(b3Vec3_zero);
		SynchronizeTriangles();
	}

	DestroyContacts();
	
	// Move the proxy so new contacts can be created.
	m_cloth->m_contactManager.m_broadPhase.TouchProxy(m_aabbProxy.broadPhaseId);
}