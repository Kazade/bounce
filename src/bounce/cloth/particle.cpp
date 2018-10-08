/*
* Copyright (c) 2016-2016 Irlan Robson http://www.irlan.net
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
#include <bounce/cloth/cloth_solver.h>
#include <bounce/cloth/dense_vec3.h>
#include <bounce/cloth/sparse_sym_mat33.h>

void b3BodyContactWorldPoint::Initialize(const b3BodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB)
{
	b3Vec3 cA = b3Mul(xfA, c->localPoint1);
	b3Vec3 cB = b3Mul(xfB, c->localPoint2);

	b3Vec3 d = cB - cA;
	float32 distance = b3Length(d);

	b3Vec3 nA(0.0f, 1.0f, 0.0f);
	if (distance > B3_EPSILON)
	{
		nA = d / distance;
	}

	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = 0.5f * (pA + pB);
	normal = nA;
	separation = distance - rA - rB;
}

void b3ParticleContactWorldPoint::Initialize(const b3ParticleContact* c)
{
	b3Vec3 cA = c->p1->GetPosition();
	float32 rA = c->p1->GetRadius();

	b3Vec3 cB = c->p2->GetPosition();
	float32 rB = c->p2->GetRadius();

	b3Vec3 d = cB - cA;
	float32 distance = b3Length(d);
	
	b3Vec3 nA(0.0f, 1.0f, 0.0f);
	if (distance > B3_EPSILON)
	{
		nA = d / distance;
	}
	
	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = 0.5f * (pA + pB);
	normal = nA;
	separation = distance - rA - rB;
}

b3Particle::b3Particle(const b3ParticleDef& def, b3Cloth* cloth)
{
	m_cloth = cloth;
	m_type = def.type;
	
	m_position = def.position;
	m_velocity = def.velocity;
	m_force = def.force;
	m_translation.SetZero();
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_radius = def.radius;
	m_friction = def.friction;
	m_userData = nullptr;
	m_x.SetZero();
	m_vertex = ~0;
}

b3Particle::~b3Particle()
{

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
	}
}