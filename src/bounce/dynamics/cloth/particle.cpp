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

#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/sparse_sym_mat33.h>

void b3FrictionForce::Apply(const b3ClothSolverData* data)
{
	// TODO
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
	m_userData = nullptr;
	m_x.SetZero();
	m_vertex = ~0;

	m_contact.n_active = false;
	m_contact.t1_active = false;
	m_contact.t2_active = false;
	m_contact.f_active = false;
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

		m_contact.f_active = false;
		m_contact.n_active = false;
		m_contact.t1_active = false;
		m_contact.t2_active = false;
	}
}