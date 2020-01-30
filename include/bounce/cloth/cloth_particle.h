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

#ifndef B3_CLOTH_PARTICLE_H
#define B3_CLOTH_PARTICLE_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/vec3.h>

class b3Cloth;

// Static particle: Zero mass. Can be moved manually.
// Kinematic particle: Zero mass. Non-zero velocity, can be moved by the solver.
// Dynamic particle: Non-zero mass. Non-zero velocity determined by force, can be moved by the solver.
enum b3ClothParticleType
{
	e_staticClothParticle,
	e_kinematicClothParticle,
	e_dynamicClothParticle
};

// Particle definition
struct b3ClothParticleDef
{
	b3ClothParticleDef()
	{
		type = e_staticClothParticle;
		position.SetZero();
		velocity.SetZero();
		force.SetZero();
		meshIndex = B3_MAX_U32;
		userData = nullptr;
	}

	b3ClothParticleType type;
	b3Vec3 position;
	b3Vec3 velocity;
	b3Vec3 force;
	u32 meshIndex;
	void* userData;
};

// A cloth particle.
class b3ClothParticle
{
public:
	// Set the particle type.
	void SetType(b3ClothParticleType type);

	// Get the particle type.
	b3ClothParticleType GetType() const;

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
	scalar GetMass() const;

	// Get the applied force.
	const b3Vec3& GetForce() const;

	// Apply a force.
	void ApplyForce(const b3Vec3& force);

	// Get the applied translation.
	const b3Vec3& GetTranslation() const;

	// Apply a translation.
	void ApplyTranslation(const b3Vec3& translation);

	// Get the mesh index.
	u32 GetMeshIndex() const;
	
	// Set the user data.
	void SetUserData(void* userData);

	// Get the user data.
	const void* GetUserData() const;
	void* GetUserData();

	// Get the next particle in the cloth list of particles.
	b3ClothParticle* GetNext();
	const b3ClothParticle* GetNext() const;
private:
	friend class b3List2<b3ClothParticle>;
	friend class b3Cloth;
	friend class b3ClothContactManager;
	friend class b3ClothSolver;
	friend class b3ClothForceSolver;
	friend class b3ClothSphereShape;
	friend class b3ClothCapsuleShape;
	friend class b3ClothTriangleShape;
	friend class b3ClothSphereAndShapeContact;
	friend class b3ClothSphereAndTriangleContact;
	friend class b3ClothCapsuleAndCapsuleContact;
	friend class b3ClothContactSolver;
	friend class b3Force;
	friend class b3StretchForce;
	friend class b3ShearForce;
	friend class b3SpringForce;
	friend class b3MouseForce;
	friend class b3ElementForce;

	b3ClothParticle(const b3ClothParticleDef& def, b3Cloth* cloth);
	~b3ClothParticle();

	// Synchronize spheres
	void SynchronizeSpheres();
	
	// Synchronize capsules
	void SynchronizeCapsules();
	
	// Synchronize triangles 
	void SynchronizeTriangles();

	// Destroy spheres.
	void DestroySpheres();
	
	// Destroy capsules.
	void DestroyCapsules();
	
	// Destroy triangles.
	void DestroyTriangles();

	// Destroy forces.
	void DestroyForces();

	// Destroy contacts.
	void DestroyContacts();

	// Type
	b3ClothParticleType m_type;

	// Position
	b3Vec3 m_position;

	// Velocity
	b3Vec3 m_velocity;

	// Applied external force
	b3Vec3 m_force;

	// Applied translation
	b3Vec3 m_translation;

	// Mass
	scalar m_mass;

	// Inverse mass
	scalar m_invMass;

	// Mesh index. 
	u32 m_meshIndex;

	// Solver temp identifier
	u32 m_solverId;

	// Solver temp solution
	b3Vec3 m_x;

	// User data
	void* m_userData;

	// Cloth
	b3Cloth* m_cloth;

	// Links to the cloth particle list.
	b3ClothParticle* m_prev;
	b3ClothParticle* m_next;
};

inline b3ClothParticleType b3ClothParticle::GetType() const
{
	return m_type;
}

inline void b3ClothParticle::SetPosition(const b3Vec3& position)
{
	m_position = position;
	m_translation.SetZero();

	SynchronizeSpheres();
	SynchronizeCapsules();
	SynchronizeTriangles();
}

inline const b3Vec3& b3ClothParticle::GetPosition() const
{
	return m_position;
}

inline void b3ClothParticle::SetVelocity(const b3Vec3& velocity)
{
	if (m_type == e_staticClothParticle)
	{
		return;
	}
	m_velocity = velocity;
}

inline const b3Vec3& b3ClothParticle::GetVelocity() const
{
	return m_velocity;
}

inline scalar b3ClothParticle::GetMass() const
{
	return m_mass;
}

inline const b3Vec3& b3ClothParticle::GetForce() const
{
	return m_force;
}

inline void b3ClothParticle::ApplyForce(const b3Vec3& force)
{
	if (m_type != e_dynamicClothParticle)
	{
		return;
	}
	m_force += force;
}

inline const b3Vec3& b3ClothParticle::GetTranslation() const
{
	return m_translation;
}

inline void b3ClothParticle::ApplyTranslation(const b3Vec3& translation)
{
	m_translation += translation;
}

inline u32 b3ClothParticle::GetMeshIndex() const
{
	return m_meshIndex;
}

inline void b3ClothParticle::SetUserData(void* userData)
{
	m_userData = userData;
}

inline const void* b3ClothParticle::GetUserData() const
{
	return m_userData;
}

inline void* b3ClothParticle::GetUserData()
{
	return m_userData;
}

inline b3ClothParticle* b3ClothParticle::GetNext()
{
	return m_next;
}

inline const b3ClothParticle* b3ClothParticle::GetNext() const
{
	return m_next;
}

#endif