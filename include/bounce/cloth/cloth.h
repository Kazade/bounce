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

#ifndef B3_CLOTH_H
#define B3_CLOTH_H

#include <bounce/common/template/list.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/math/transform.h>
#include <bounce/cloth/cloth_contact_manager.h>

class b3World;

struct b3ParticleDef;
class b3Particle;

struct b3ForceDef;
class b3Force;

class b3ClothTriangle;

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
		streching = 0.0f;
		shearing = 0.0f;
		bending = 0.0f;
		sewing = 0.0f;
		damping = 0.0f;
		thickness = 0.0f;
		friction = 0.2f;
	}

	// Cloth mesh 
	const b3ClothMesh* mesh;

	// Cloth density in kg/m^2
	float32 density;

	// Streching stiffness
	float32 streching;

	// Shearing stiffness
	float32 shearing;
	
	// Bending stiffness
	float32 bending;

	// Sewing stiffness
	float32 sewing;
	
	// Damping stiffness
	float32 damping;

	// Cloth thickness
	float32 thickness;

	// Cloth coefficient of friction
	float32 friction;
};

// A cloth represents a deformable surface as a collection of particles.
// Particles may be connected with each other by springs.
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
	// The cloth will be able to respond to collisions with the rigid bodies in the attached world.
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

	// Return the cloth particle given the vertex index.
	b3Particle* GetParticle(u32 i);

	// Return the cloth triangle given the triangle index.
	b3ClothTriangle* GetTriangle(u32 i);

	// Return the list of particles in this cloth.
	const b3List2<b3Particle>& GetParticleList() const;

	// Return the list of forces in this cloth.
	const b3List2<b3Force>& GetForceList() const;

	// Return the kinetic (or dynamic) energy in this system.
	float32 GetEnergy() const;

	// Perform a time step. 
	void Step(float32 dt, u32 velocityIterations, u32 positionIterations);

	// Debug draw the cloth using the associated cloth mesh.
	void Draw() const;
private:
	friend class b3Particle;
	friend class b3ClothTriangle;
	friend class b3ShearForce;
	friend class b3StrechForce;
	friend class b3SpringForce;
	friend class b3MouseForce;
	friend class b3ClothContactManager;

	// Compute mass of each particle.
	void ComputeMass();

	// Solve
	void Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// The world attached to this cloth
	b3World* m_world;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Proxy mesh
	const b3ClothMesh* m_mesh;
	
	// Particles
	b3Particle** m_particles;

	// Triangles
	b3ClothTriangle* m_triangles;

	// Cloth density
	float32 m_density;

	// Pool of particles
	b3BlockPool m_particleBlocks;

	// List of particles
	b3List2<b3Particle> m_particleList;

	// List of forces
	b3List2<b3Force> m_forceList;

	// Contact manager
	b3ClothContactManager m_contactManager;
};

inline void b3Cloth::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3Cloth::GetGravity() const
{
	return m_gravity;
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