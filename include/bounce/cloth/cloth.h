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

struct b3ClothMesh;

struct b3ClothParticleDef;
class b3ClothParticle;

struct b3ForceDef;
class b3Force;

struct b3ClothSphereShapeDef;
class b3ClothSphereShape;

struct b3ClothCapsuleShapeDef;
class b3ClothCapsuleShape;

struct b3ClothTriangleShapeDef;
class b3ClothTriangleShape;

struct b3ClothWorldShapeDef;
class b3ClothWorldShape;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3ClothTimeStep;

struct b3ClothRayCastSingleOutput
{
	b3ClothTriangleShape* triangle;
	scalar fraction;
	b3Vec3 normal;
};

// Cloth definition
// This requires defining a cloth mesh which is typically bound to a render mesh 
// and some uniform parameters.
struct b3ClothDef
{
	b3ClothDef()
	{
		mesh = nullptr;
		density = scalar(0);
		streching = scalar(0);
		strechDamping = scalar(0);
		shearing = scalar(0);
		shearDamping = scalar(0);
		bending = scalar(0);
		bendDamping = scalar(0);
		sewing = scalar(0);
		sewDamping = scalar(0);
		thickness = scalar(0);
		friction = scalar(0.2);
	}

	// Cloth mesh 
	const b3ClothMesh* mesh;

	// Cloth density in kg/m^2
	scalar density;

	// Streching stiffness
	scalar streching;

	// Strech damping stiffness
	scalar strechDamping;
	
	// Shearing stiffness
	scalar shearing;

	// Shear damping stiffness
	scalar shearDamping;
	
	// Bending stiffness
	scalar bending;

	// Bend damping stiffness
	scalar bendDamping;
	
	// Sewing stiffness
	scalar sewing;
	
	// Sew damping stiffness
	scalar sewDamping;

	// Shape thickness
	scalar thickness;

	// Shape coefficient of friction
	scalar friction;
};

// A cloth represents a deformable surface as a collection of particles.
// Particles may be connected with each other by forces.
class b3Cloth
{
public:
	b3Cloth();
	b3Cloth(const b3ClothDef& def);
	~b3Cloth();

	// Create a particle.
	b3ClothParticle* CreateParticle(const b3ClothParticleDef& def);

	// Destroy a given particle.
	void DestroyParticle(b3ClothParticle* particle);

	// Return the list of particles in this cloth.
	const b3List2<b3ClothParticle>& GetParticleList() const;

	// Create a force.
	b3Force* CreateForce(const b3ForceDef& def);

	// Destroy a given force.
	void DestroyForce(b3Force* force);

	// Return the list of forces in this cloth.
	const b3List2<b3Force>& GetForceList() const;

	// Create a sphere shape.
	b3ClothSphereShape* CreateSphereShape(const b3ClothSphereShapeDef& def);

	// Destroy a given sphere shape.
	void DestroySphereShape(b3ClothSphereShape* shape);
	
	// Return the list of sphere shapes in this cloth.
	const b3List2<b3ClothSphereShape>& GetSphereShapeList() const;
	
	// Create a capsule shape.
	b3ClothCapsuleShape* CreateCapsuleShape(const b3ClothCapsuleShapeDef& def);

	// Destroy a given capsule shape.
	void DestroyCapsuleShape(b3ClothCapsuleShape* shape);

	// Return the list of capsule shapes in this cloth.
	const b3List2<b3ClothCapsuleShape>& GetCapsuleShapeList() const;

	// Create a triangle shape.
	b3ClothTriangleShape* CreateTriangleShape(const b3ClothTriangleShapeDef& def);

	// Destroy a given triangle shape.
	void DestroyTriangleShape(b3ClothTriangleShape* shape);

	// Return the list of triangle shapes in this cloth.
	const b3List2<b3ClothTriangleShape>& GetTriangleShapeList() const;

	// Create a new world shape.
	b3ClothWorldShape* CreateWorldShape(const b3ClothWorldShapeDef& def);

	// Destroy a given world shape.
	void DestroyWorldShape(b3ClothWorldShape* shape);

	// Return the list of world shapes in this cloth.
	const b3List2<b3ClothWorldShape>& GetWorldShapeList() const;

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Perform a time step. 
	void Step(scalar dt, u32 velocityIterations, u32 positionIterations);

	// Perform a ray cast with the cloth.
	bool RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Get the cloth mesh.
	const b3ClothMesh* GetMesh() const;

	// Get mesh particle.
	b3ClothParticle* GetParticle(u32 index);
	
	// Get mesh sphere.
	b3ClothSphereShape* GetSphere(u32 index);

	// Get mesh triangle.
	b3ClothTriangleShape* GetTriangle(u32 index);

	// Enable or disable self-collision.
	void EnableSelfCollision(bool flag);

	// Is self-collision enabled?
	bool IsSelfCollisionEnabled() const;

	// Return the kinetic (or dynamic) energy in this system.
	scalar GetEnergy() const;

	// Debug draw the cloth entities.
	void Draw() const;
private:
	friend class b3ClothParticle;
	friend class b3ClothSphereShape;
	friend class b3ClothCapsuleShape;
	friend class b3ClothTriangleShape;
	friend class b3ClothWorldShape;
	friend class b3ShearForce;
	friend class b3StretchForce;
	friend class b3SpringForce;
	friend class b3MouseForce;
	friend class b3ClothContactManager;
	
	// Rest the mass data of the cloth.
	void ResetMass();

	// Solve
	void Solve(const b3ClothTimeStep& step);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Pool of particles
	b3BlockPool m_particleBlocks;

	// Pool of sphere shapes
	b3BlockPool m_sphereShapeBlocks;
	
	// Pool of capsule shapes
	b3BlockPool m_capsuleShapeBlocks;
	
	// Pool of triangle shapes
	b3BlockPool m_triangleShapeBlocks;
	
	// Pool of world shapes
	b3BlockPool m_worldShapeBlocks;
	
	// List of particles
	b3List2<b3ClothParticle> m_particleList;

	// List of forces
	b3List2<b3Force> m_forceList;

	// List of sphere shapes
	b3List2<b3ClothSphereShape> m_sphereShapeList;
	
	// List of capsule shapes
	b3List2<b3ClothCapsuleShape> m_capsuleShapeList;
	
	// List of triangle shapes
	b3List2<b3ClothTriangleShape> m_triangleShapeList;
	
	// List of world shapes
	b3List2<b3ClothWorldShape> m_worldShapeList;

	// Contact manager
	b3ClothContactManager m_contactManager;

	// Used to compute the time step ratio to 
	// support variable time steps.
	scalar m_inv_dt0;

	// Mesh
	const b3ClothMesh* m_mesh;

	// Mesh vertex particles
	b3ClothParticle** m_particles;
	
	// Mesh vertex sphere shapes
	b3ClothSphereShape** m_spheres;

	// Mesh triangle triangle shapes
	b3ClothTriangleShape** m_triangles;

	// Self-collision activation flag
	bool m_enableSelfCollision;
};

inline void b3Cloth::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3Cloth::GetGravity() const
{
	return m_gravity;
}

inline bool b3Cloth::IsSelfCollisionEnabled() const
{
	return m_enableSelfCollision;
}

inline const b3List2<b3Force>& b3Cloth::GetForceList() const
{
	return m_forceList;
}

inline const b3List2<b3ClothParticle>& b3Cloth::GetParticleList() const
{
	return m_particleList;
}

inline const b3List2<b3ClothSphereShape>& b3Cloth::GetSphereShapeList() const
{
	return m_sphereShapeList;
}

inline const b3List2<b3ClothCapsuleShape>& b3Cloth::GetCapsuleShapeList() const
{
	return m_capsuleShapeList;
}

inline const b3List2<b3ClothTriangleShape>& b3Cloth::GetTriangleShapeList() const
{
	return m_triangleShapeList;
}

inline const b3List2<b3ClothWorldShape>& b3Cloth::GetWorldShapeList() const
{
	return m_worldShapeList;
}

inline const b3ClothMesh* b3Cloth::GetMesh() const
{
	return m_mesh;
}

#endif