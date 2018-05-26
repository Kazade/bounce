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

#ifndef B3_CLOTH_H
#define B3_CLOTH_H

#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>

class b3StackAllocator;
class b3World;
class b3Shape;

struct b3ClothMesh;

// Cloth mesh definition
struct b3ClothDef
{
	b3ClothDef()
	{
		mesh = nullptr;
		density = 0.0f;
		r = 0.05f;
		ks = 0.0f;
		kb = 0.0f;
		kd = 0.0f;
	}

	// Cloth proxy mesh 
	b3ClothMesh* mesh;

	// Radius
	// This should be a small value. It can be used for correcting visual artifacts when 
	// the masses are colliding against a solid.
	float32 r;

	// Cloth density in kg/m^3
	float32 density;

	// Streching stiffness
	float32 ks;

	// Bending stiffness
	float32 kb;

	// Damping stiffness
	float32 kd;
};

// Static particle: Has zero mass, can be moved manually.
// Kinematic particle: Has zero mass, non-zero velocity, can be moved by the solver.
// Dynamic particle: Has non-zero mass, non-zero velocity determined by force, can be moved by the solver.
enum b3ParticleType
{
	e_staticParticle,
	e_kinematicParticle,
	e_dynamicParticle
};

// Read-only particle 
struct b3Particle
{
	// Type
	b3ParticleType type;

	// Position
	b3Vec3 position;

	// Velocity
	b3Vec3 velocity;
	
	// Applied external force
	b3Vec3 force;
	
	// Mass
	float32 mass;
	
	// Inverse mass
	float32 invMass;
	
	// Radius
	float32 radius;

	// User data. 
	void* userData;

	// Applied external translation
	b3Vec3 translation;

	// Solver temp

	// Identifier
	u32 solverId;
	
	// Solution
	b3Vec3 x;
};

// Spring types
enum b3SpringType
{
	e_strechSpring,
	e_bendSpring,
};

struct b3ClothSolverData;

// Read-only spring
struct b3Spring
{
	// Solver shared
	
	// Spring type
	b3SpringType type;

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Rest length
	float32 L0;

	// Structural stiffness
	float32 ks;
	
	// Damping stiffness
	float32 kd;

	// Solver temp

	// Action tensile force (f_i entry)
	b3Vec3 tension;

	// Jacobian (J_ii entry)
	b3Mat33 Jx, Jv;

	// Apply spring forces.
	void ApplyForces(const b3ClothSolverData* data);
};

// Read-only body contact between a particle and a solid
struct b3BodyContact
{
	b3Particle* p1;
	b3Shape* s2;
	float32 s;
	b3Vec3 n, t1, t2;
	float32 Fn, Ft1, Ft2;
	bool n_active, t1_active, t2_active;
};

// A cloth represents a deformable surface/mesh.
// b3Cloth simulates this surface motion using particles and springs.
class b3Cloth
{
public:
	// Get the world the cloth belongs to.
	const b3World* GetWorld() const;
	b3World* GetWorld();

	// Return the cloth mesh used to initialize this cloth.
	b3ClothMesh* GetMesh() const;

	// Return the number of particles in this cloth.
	u32 GetParticleCount() const;

	// Return the particle at a given index in this cloth.
	b3Particle* GetParticle(u32 i) const;
	
	// Convenience function.
	// Return the index of a given particle.
	u32 GetParticleIndex(const b3Particle* p) const;

	// Set the type of a given particle.
	void SetType(b3Particle* p, b3ParticleType type);

	// Translate a given particle in the next time step.
	void Translate(b3Particle* p, const b3Vec3& translation);

	// Set the velocity of a given particle.
	void SetVelocity(b3Particle* p, const b3Vec3& velocity);

	// Apply a force to a given particle.
	void ApplyForce(b3Particle* p, const b3Vec3& force);

	// Return the number of springs in this cloth.
	u32 GetSpringCount() const;

	// Return the spring at a given index in this cloth.
	b3Spring* GetSpring(u32 i) const;
	
	// Return the kinetic (or dynamic) energy in this system.
	float32 GetEnergy() const;

	// Get the next cloth in the world cloth list.
	const b3Cloth* GetNext() const;

	// Get the next cloth in the world cloth list.
	b3Cloth* GetNext();

	// Set the positions of the mesh vertices to the positions of their associated particles.
	void Apply() const;
	
	// Debug draw the cloth using the associated cloth mesh.
	void Draw() const;
private:
	friend class b3World;

	friend class b3List2<b3Cloth>;

	b3Cloth(const b3ClothDef& def, b3World* world);
	~b3Cloth();

	// Perform a time step. Called only inside b3World.
	void Step(float32 dt, const b3Vec3& gravity);

	// Compute mass of each particle.
	void ResetMass();

	// Update contacts. 
	// This is where some contacts might be initiated or terminated.
	void UpdateContacts();

	// Solve
	void Solve(float32 dt, const b3Vec3& gravity);

	b3StackAllocator* m_allocator;

	b3ClothMesh* m_mesh;
	float32 m_density;

	u32 m_particleCount;
	b3Particle* m_particles;

	u32 m_springCount;
	b3Spring* m_springs;

	b3BodyContact* m_contacts;
	//u32 m_contactCount;

	// The parent world of this cloth.
	b3World* m_world;
	
	// Links to the world cloth list.
	b3Cloth* m_prev;
	b3Cloth* m_next;
};

inline const b3World* b3Cloth::GetWorld() const
{
	return m_world;
}

inline b3World* b3Cloth::GetWorld()
{
	return m_world;
}

inline b3ClothMesh* b3Cloth::GetMesh() const
{
	return m_mesh;
}

inline u32 b3Cloth::GetParticleCount() const
{
	return m_particleCount;
}

inline b3Particle* b3Cloth::GetParticle(u32 i) const
{
	B3_ASSERT(i < m_particleCount);
	return m_particles + i;
}

inline u32 b3Cloth::GetParticleIndex(const b3Particle* p) const
{
	return u32(p - m_particles);
}

inline void b3Cloth::SetType(b3Particle* p, b3ParticleType type)
{
	if (p->type == type)
	{
		return;
	}

	p->type = type;
	p->force.SetZero();
	
	if (type == e_staticParticle)
	{
		p->velocity.SetZero();
		p->translation.SetZero();

		u32 ip = u32(p - m_particles);

		m_contacts[ip].n_active = false;
		m_contacts[ip].t1_active = false;
		m_contacts[ip].t2_active = false;
	}
}

inline void b3Cloth::Translate(b3Particle* p, const b3Vec3& translation)
{
	p->translation += translation;
}

inline void b3Cloth::SetVelocity(b3Particle* p, const b3Vec3& velocity)
{
	if (p->type == e_staticParticle)
	{
		return;
	}
	p->velocity = velocity;
}

inline void b3Cloth::ApplyForce(b3Particle* p, const b3Vec3& force)
{
	if (p->type != e_dynamicParticle)
	{
		return;
	}
	p->force += force;
}

inline u32 b3Cloth::GetSpringCount() const
{
	return m_springCount;
}

inline b3Spring* b3Cloth::GetSpring(u32 i) const
{
	B3_ASSERT(i < m_springCount);
	return m_springs + i;
}

inline float32 b3Cloth::GetEnergy() const
{
	float32 E = 0.0f;
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		E += m_particles[i].mass * b3Dot(m_particles[i].velocity, m_particles[i].velocity);
	}
	return 0.5f * E;
}

inline const b3Cloth* b3Cloth::GetNext() const
{
	return m_next;
}

inline b3Cloth* b3Cloth::GetNext()
{
	return m_next;
}

#endif