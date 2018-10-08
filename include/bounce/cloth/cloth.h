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
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_pool.h>

class b3World;
class b3Shape;

class b3Particle;
class b3Force;
class b3BodyContact;
class b3ParticleContact;
class b3TriangleContact;

struct b3ParticleDef;
struct b3ForceDef;

struct b3ClothMesh;

class b3RayCastListener;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3ClothRayCastSingleOutput
{
	u32 triangle;
	float32 fraction;
	b3Vec3 normal;
};

// Cloth definition
// This requires defining a cloth mesh which is typically bound to a render mesh
struct b3ClothDef
{
	b3ClothDef()
	{
		mesh = nullptr;
		density = 0.0f;
		structural = 0.0f;
		bending = 0.0f;
		damping = 0.0f;
	}

	// Cloth mesh 
	const b3ClothMesh* mesh;

	// Cloth density in kg/m^3
	float32 density;

	// Structural stiffness
	float32 structural;

	// Bending stiffness
	float32 bending;

	// Damping stiffness
	float32 damping;
};

// A cloth represents a deformable surface as a collection of particles.
// Particles may be connected with each other.
class b3Cloth
{
public:
	b3Cloth(const b3ClothDef& def);
	~b3Cloth();

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Attach a world to this cloth. 
	// The cloth will be able to respond to collisions with the static shapes in the attached world.
	void SetWorld(b3World* world);

	// Get the world attached to this cloth.
	const b3World* GetWorld() const;
	b3World* GetWorld();

	// Create a particle.
	b3Particle* CreateParticle(const b3ParticleDef& def);

	// Destroy a given particle.
	void DestroyParticle(b3Particle* particle);

	// Create a force.
	b3Force* CreateForce(const b3ForceDef& def);

	// Destroy a given force.
	void DestroyForce(b3Force* force);

	// Perform a ray cast with the cloth.
	bool RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Perform a ray cast with a given cloth mesh triangle.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput* input, u32 triangleIndex) const;

	// Return the cloth mesh proxy.
	const b3ClothMesh* GetMesh() const;

	// Return the particle associated with the given vertex.
	b3Particle* GetVertexParticle(u32 i);

	// Return the list of particles in this cloth.
	const b3List2<b3Particle>& GetParticleList() const;

	// Return the list of forces in this cloth.
	const b3List2<b3Force>& GetForceList() const;

	// Return the kinetic (or dynamic) energy in this system.
	float32 GetEnergy() const;

	// Get the next cloth in the world cloth list.
	const b3Cloth* GetNext() const;

	// Get the next cloth in the world cloth list.
	b3Cloth* GetNext();

	// Perform a time step. 
	void Step(float32 dt);

	// Debug draw the cloth using the associated cloth mesh.
	void Draw() const;
private:
	friend class b3List2<b3Cloth>;

	// Compute mass of each particle.
	void ComputeMass();

	// Update body contacts. 
	void UpdateBodyContacts();

	// Update particle contacts. 
	void UpdateParticleContacts();
	
	// Update triangle contacts. 
	void UpdateTriangleContacts();

	// Update contacts
	void UpdateContacts();

	// Solve
	void Solve(float32 dt, const b3Vec3& gravity);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// The world attached to this cloth
	b3World* m_world;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Proxy mesh
	const b3ClothMesh* m_mesh;
	
	// Vertex particles
	b3Particle** m_vertexParticles;

	// Cloth density
	float32 m_density;

	// Pool of particles
	b3BlockPool m_particleBlocks;

	// Pool of body contacts
	b3BlockPool m_bodyContactBlocks;
	
	// Pool of particle contacts
	b3BlockPool m_particleContactBlocks;
	
	// Pool of triangle contacts
	b3BlockPool m_triangleContactBlocks;
	
	// List of particles
	b3List2<b3Particle> m_particleList;
	
	// List of forces
	b3List2<b3Force> m_forceList;

	// List of particle contacts
	b3List2<b3BodyContact> m_bodyContactList;
	
	// List of particle contacts
	b3List2<b3ParticleContact> m_particleContactList;
	
	// List of triangle contacts
	b3List2<b3TriangleContact> m_triangleContactList;
};

inline void b3Cloth::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3Cloth::GetGravity() const
{
	return m_gravity;
}

inline void b3Cloth::SetWorld(b3World* world)
{
	m_world = world;
}

inline const b3World* b3Cloth::GetWorld() const
{
	return m_world;
}

inline b3World* b3Cloth::GetWorld()
{
	return m_world;
}

inline const b3ClothMesh* b3Cloth::GetMesh() const
{
	return m_mesh;
}

inline const b3List2<b3Particle>& b3Cloth::GetParticleList() const
{
	return m_particleList;
}

inline const b3List2<b3Force>& b3Cloth::GetForceList() const
{
	return m_forceList;
}

#endif