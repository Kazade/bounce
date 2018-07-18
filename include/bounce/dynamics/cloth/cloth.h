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
#include <bounce/common/memory/block_pool.h>

class b3World;
class b3Shape;

class b3Particle;
class b3Force;

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
	b3ClothMesh* mesh;

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
	// Get the world the cloth belongs to.
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
	void RayCast(b3RayCastListener* listener, const b3Vec3& p1, const b3Vec3& p2);

	// Perform a ray cast with the cloth.
	bool RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Perform a ray cast with a given cloth mesh triangle.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput* input, u32 triangleIndex) const;

	// Return the cloth mesh proxy.
	b3ClothMesh* GetMesh() const;

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

	// Debug draw the cloth using the associated cloth mesh.
	void Draw() const;
private:
	friend class b3World;

	friend class b3List2<b3Cloth>;

	b3Cloth(const b3ClothDef& def, b3World* world);
	~b3Cloth();

	// Perform a time step. 
	// Called only inside b3World.
	void Step(float32 dt, const b3Vec3& gravity);

	// Compute mass of each particle.
	void ComputeMass();

	// Update contacts. 
	// This is where some contacts might be initiated or terminated.
	void UpdateContacts();

	// Solve
	void Solve(float32 dt, const b3Vec3& gravity);

	// Proxy mesh
	b3ClothMesh* m_mesh;
	
	// Mesh density
	float32 m_density;

	// Pool of particles
	b3BlockPool m_particleBlocks;

	// List of particles
	b3List2<b3Particle> m_particleList;
	
	// List of forces
	b3List2<b3Force> m_forceList;

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

inline const b3List2<b3Particle>& b3Cloth::GetParticleList() const
{
	return m_particleList;
}

inline const b3List2<b3Force>& b3Cloth::GetForceList() const
{
	return m_forceList;
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