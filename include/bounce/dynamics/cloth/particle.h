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

#ifndef B3_PARTICLE_H
#define B3_PARTICLE_H

#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>
#include <bounce/dynamics/cloth/force.h>

class b3Shape;
class b3Cloth;
class b3Particle;

// Static particle: Can be moved manually.
// Kinematic particle: Non-zero velocity, can be moved by the solver.
// Dynamic particle: Non-zero velocity determined by force, can be moved by the solver.
enum b3ParticleType
{
	e_staticParticle,
	e_kinematicParticle,
	e_dynamicParticle
};

//
struct b3ParticleDef
{
	b3ParticleDef()
	{
		type = e_staticParticle;
		position.SetZero();
		velocity.SetZero();
		force.SetZero();
		radius = 0.0f;
		userData = nullptr;
	}

	b3ParticleType type;
	b3Vec3 position;
	b3Vec3 velocity;
	b3Vec3 force;
	float32 radius;
	void* userData;
};

//
class b3FrictionForce : public b3Force
{
public:
	b3FrictionForce() { }
	~b3FrictionForce() { }
	
	void Apply(const b3ClothSolverData* data);

	b3Particle* m_p;
};

// A contact between a particle and a solid
class b3BodyContact
{
public:
	b3BodyContact() { }
	~b3BodyContact() { }

	b3Particle* p1;
	b3Shape* s2;
	float32 s;

	bool f_active;
	b3FrictionForce f;
	
	bool n_active, t1_active, t2_active;
	b3Vec3 n, t1, t2;
	float32 Fn, Ft1, Ft2;
};

// A cloth particle.
class b3Particle
{
public:
	// Set the particle type.
	void SetType(b3ParticleType type);

	// Get the particle type.
	b3ParticleType GetType() const;

	// Get the vertex index.
	u32 GetVertex() const;

	// Set the particle position. 
	// If the particle is dynamic changing the position directly might lead 
	// to physically incorrect simulation behaviour.
	void SetPosition(const b3Vec3& position);

	// Get the particle position.
	const b3Vec3& GetPosition() const;

	// Set the particle velocity.
	void SetVelocity(const b3Vec3& velocity);

	// Get the particle velocity.
	const b3Vec3& GetVelocity() const;

	// Get the particle mass.
	float32 GetMass() const;

	// Get the particle radius;
	float32 GetRadius() const;

	// Apply a force.
	void ApplyForce(const b3Vec3& force);

	// Apply a translation.
	void ApplyTranslation(const b3Vec3& translation);

	// Get the next particle.
	b3Particle* GetNext();
private:
	friend class b3List2<b3Particle>;
	friend class b3Cloth;
	friend class b3ClothSolver;
	friend class b3Force;
	friend class b3SpringForce;
	friend class b3FrictionForce;

	b3Particle(const b3ParticleDef& def, b3Cloth* cloth);
	~b3Particle();

	// Type
	b3ParticleType m_type;

	// Position
	b3Vec3 m_position;

	// Velocity
	b3Vec3 m_velocity;

	// Applied external force
	b3Vec3 m_force;

	// Mass
	float32 m_mass;

	// Inverse mass
	float32 m_invMass;

	// Radius
	float32 m_radius;

	// User data. 
	void* m_userData;

	// Cloth mesh vertex index.
	u32 m_vertex;

	// Applied external translation
	b3Vec3 m_translation;

	// Contact
	b3BodyContact m_contact;

	// Solver temp

	// Identifier
	u32 m_solverId;

	// Solution
	b3Vec3 m_x;

	// 
	b3Cloth* m_cloth;

	// 
	b3Particle* m_prev;

	//
	b3Particle* m_next;
};

inline b3ParticleType b3Particle::GetType() const
{
	return m_type;
}

inline u32 b3Particle::GetVertex() const
{
	return m_vertex;
}

inline void b3Particle::SetPosition(const b3Vec3& position)
{
	m_position = position;
	m_translation.SetZero();
}

inline const b3Vec3& b3Particle::GetPosition() const
{
	return m_position;
}

inline void b3Particle::SetVelocity(const b3Vec3& velocity)
{
	if (m_type == e_staticParticle)
	{
		return;
	}
	m_velocity = velocity;
}

inline const b3Vec3& b3Particle::GetVelocity() const
{
	return m_velocity;
}

inline float32 b3Particle::GetMass() const
{
	return m_mass;
}

inline float32 b3Particle::GetRadius() const
{
	return m_radius;
}

inline void b3Particle::ApplyForce(const b3Vec3& force)
{
	if (m_type != e_dynamicParticle)
	{
		return;
	}
	m_force += force;
}

inline void b3Particle::ApplyTranslation(const b3Vec3& translation)
{
	m_translation += translation;
}

inline b3Particle* b3Particle::GetNext()
{
	return m_next;
}

#endif