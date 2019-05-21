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

#ifndef B3_SOFT_BODY_H
#define B3_SOFT_BODY_H

#include <bounce/common/math/transform.h>
#include <bounce/common/memory/stack_allocator.h>

class b3World;

struct b3SoftBodyMesh;

class b3SoftBodyNode;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3SoftBodyRayCastSingleOutput
{
	u32 tetrahedron;
	u32 v1, v2, v3;
	float32 fraction;
	b3Vec3 normal;
};

// Soft body tetrahedron element
struct b3SoftBodyElement
{
	b3Mat33 K[16];
	b3Mat33 invE;
	b3Quat q;
};

// Soft body tetrahedron triangle
struct b3SoftBodyTriangle
{
	u32 v1, v2, v3;
	u32 tetrahedron;
};

// Soft body definition
// This requires defining a soft body mesh which is typically bound to a render mesh
struct b3SoftBodyDef
{
	b3SoftBodyDef()
	{
		mesh = nullptr;
		density = 0.1f;
		E = 100.0f;
		nu = 0.3f;
	}

	// Soft body mesh
	const b3SoftBodyMesh* mesh;

	// Density in kg/m^3
	float32 density;

	// Material Young's modulus in [0, inf]
	// Units are 1e3N/m^2
	float32 E;

	// Material Poisson ratio in [0, 0.5]
	// This is a dimensionless value
	float32 nu;
};

// A soft body represents a deformable volume as a collection of nodes and elements.
class b3SoftBody
{
public:
	b3SoftBody(const b3SoftBodyDef& def);
	~b3SoftBody();

	// Perform a ray cast with the soft body.
	bool RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Attach a world to this soft body. 
	// The soft body will be able to respond to collisions with the bodies in the attached world.
	void SetWorld(b3World* world);

	// Get the world attached to this soft body.
	const b3World* GetWorld() const;
	b3World* GetWorld();

	// Return the soft body mesh proxy.
	const b3SoftBodyMesh* GetMesh() const;

	// Return the node associated with the given vertex.
	b3SoftBodyNode* GetVertexNode(u32 i);

	// Return the kinetic (or dynamic) energy in this system.
	float32 GetEnergy() const;

	// Perform a time step. 
	void Step(float32 dt, u32 velocityIterations, u32 positionIterations);

	// Debug draw the body using the associated mesh.
	void Draw() const;
private:
	// Compute mass of each node.
	void ComputeMass();

	// Update contacts.
	void UpdateContacts();

	// Solve
	void Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Proxy mesh
	const b3SoftBodyMesh* m_mesh;

	// Soft body density
	float32 m_density;

	// Material Young's modulus
	float32 m_E;

	// Material poisson ratio
	float32 m_nu;

	// Soft body nodes
	b3SoftBodyNode* m_nodes;

	// Soft body elements
	b3SoftBodyElement* m_elements;

	// Soft body triangles
	b3SoftBodyTriangle* m_triangles;

	// Attached world
	b3World* m_world;
};

inline void b3SoftBody::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3SoftBody::GetGravity() const
{
	return m_gravity;
}

inline void b3SoftBody::SetWorld(b3World* world)
{
	m_world = world;
}

inline const b3World* b3SoftBody::GetWorld() const
{
	return m_world;
}

inline b3World* b3SoftBody::GetWorld()
{
	return m_world;
}

inline const b3SoftBodyMesh* b3SoftBody::GetMesh() const
{
	return m_mesh;
}

#endif