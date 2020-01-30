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

#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_pool.h>
#include <bounce/common/template/list.h>
#include <bounce/softbody/softbody_contact_manager.h>

struct b3SoftBodyMesh;

struct b3SoftBodyTimeStep;

class b3SoftBodyNode;
class b3SoftBodyElement;

struct b3SoftBodySphereShapeDef;
class b3SoftBodySphereShape;

struct b3SoftBodyWorldShapeDef;
class b3SoftBodyWorldShape;

struct b3SoftBodyAnchorDef;
class b3SoftBodyAnchor;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3SoftBodyRayCastSingleOutput
{
	u32 triangle;
	scalar fraction;
	b3Vec3 normal;
};

struct b3SparseMat33Pattern;

// Soft body definition
// This requires defining a soft body mesh which is typically bound to a render mesh
// and some uniform material parameters.
struct b3SoftBodyDef
{
	b3SoftBodyDef()
	{
		mesh = nullptr;
		density = scalar(0.1);
		E = scalar(100);
		nu = scalar(0.3);
		c_yield = B3_MAX_SCALAR;
		c_creep = scalar(0);
		c_max = scalar(0);
		radius = scalar(0);
		friction = scalar(0.2);
		massDamping = scalar(0);
		stiffnessDamping = scalar(0);
	}

	// Soft body mesh
	const b3SoftBodyMesh* mesh;

	// Density in kg/m^3
	scalar density;

	// Material Young's modulus in [0, inf]
	// Units are 1e3N/m^2
	scalar E;

	// Material Poisson ratio in [0, 0.5]
	// This is a dimensionless value
	scalar nu;

	// Material elastic strain yield in [0, inf]
	// This is a dimensionless value
	scalar c_yield;

	// Material creep rate in [0, 1 / dt]
	// Units are Hz
	scalar c_creep;

	// Material maximum plastic strain in [0, inf]
	// This is a dimensionless value
	scalar c_max;

	// Soft body radius
	scalar radius;

	// Soft body coefficient of friction
	scalar friction;

	// Soft body mass damping coefficient
	scalar massDamping;

	// Soft body stiffness damping coefficient
	scalar stiffnessDamping;
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
	const b3Vec3& GetGravity() const;

	// Set the coefficient of mass damping.
	void SetMassDamping(scalar damping);

	// Get the coefficient of mass damping.
	scalar GetMassDamping() const;

	// Set the coefficient of stiffness damping.
	void SetStiffnessDamping(scalar damping);

	// Get the coefficient of stiffness damping.
	scalar GetStiffnessDamping() const;

	// Create a sphere shape.
	b3SoftBodySphereShape* CreateSphereShape(const b3SoftBodySphereShapeDef& def);

	// Destroy a given sphere shape.
	void DestroySphereShape(b3SoftBodySphereShape* shape);

	// Return the list of sphere shapes.
	b3List2<b3SoftBodySphereShape>& GetSphereShapeList();
	const b3List2<b3SoftBodySphereShape>& GetSphereShapeList() const;

	// Create a world shape.
	b3SoftBodyWorldShape* CreateWorldShape(const b3SoftBodyWorldShapeDef& def);

	// Destroy a given world shape.
	void DestroyWorldShape(b3SoftBodyWorldShape* shape);

	// Return the list of world shapes.
	b3List2<b3SoftBodyWorldShape>& GetWorldShapeList();
	const b3List2<b3SoftBodyWorldShape>& GetWorldShapeList() const;

	// Create a soft body anchor joint.
	b3SoftBodyAnchor* CreateAnchor(const b3SoftBodyAnchorDef& def);

	// Destroy a given soft body anchor joint.
	void DestroyAnchor(b3SoftBodyAnchor* anchor);

	// Get the list of anchor joints in this soft body.
	const b3List2<b3SoftBodyAnchor>& GetAnchorList() const;
	b3List2<b3SoftBodyAnchor>& GetAnchorList();

	// Return the soft body mesh proxy.
	const b3SoftBodyMesh* GetMesh() const;

	// Return the node associated with the given vertex.
	b3SoftBodyNode* GetNode(u32 i);

	// Return the element associated with the given tetrahedron.
	b3SoftBodyElement* GetElement(u32 i);

	// Return the kinetic (or dynamic) energy in this system.
	scalar GetEnergy() const;

	// Perform a time step. 
	void Step(scalar dt, u32 velocityIterations, u32 positionIterations);

	// Debug draw the body using the associated mesh.
	void Draw() const;
private:
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyForceSolver;
	friend class b3SoftBodyNode;
	friend class b3SoftBodyElement;
	friend class b3SoftBodySphereShape;
	friend class b3SoftBodyWorldShape;

	// Compute mass of each node.
	void ComputeMass();

	// Solve
	void Solve(const b3SoftBodyTimeStep& step);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Pool of sphere shapes
	b3BlockPool m_sphereShapeBlocks;
	
	// Pool of world shapes
	b3BlockPool m_worldShapeBlocks;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// Soft body density
	scalar m_density;

	// Mass damping coefficient
	scalar m_massDamping;
	
	// Stiffness damping coefficient
	scalar m_stiffnessDamping;

	// Proxy mesh
	const b3SoftBodyMesh* m_mesh;

	// Soft body nodes
	b3SoftBodyNode* m_nodes;

	// Soft body elements
	b3SoftBodyElement* m_elements;

	// Contact manager
	b3SoftBodyContactManager m_contactManager;

	// Anchor joints
	b3List2<b3SoftBodyAnchor> m_anchorList;

	// Sphere shapes
	b3List2<b3SoftBodySphereShape> m_sphereShapeList;

	// World shapes
	b3List2<b3SoftBodyWorldShape> m_worldShapeList;

	// Stiffness matrix sparsity pattern
	b3SparseMat33Pattern* m_KP;

	// Used to compute the ration to support
	// variable time steps
	scalar m_inv_dt0;
};

inline void b3SoftBody::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline const b3Vec3& b3SoftBody::GetGravity() const
{
	return m_gravity;
}

inline void b3SoftBody::SetMassDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_massDamping = damping;
}

inline scalar b3SoftBody::GetMassDamping() const
{
	return m_massDamping;
}

inline void b3SoftBody::SetStiffnessDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_stiffnessDamping = damping;
}

inline scalar b3SoftBody::GetStiffnessDamping() const
{
	return m_stiffnessDamping;
}

inline b3List2<b3SoftBodySphereShape>& b3SoftBody::GetSphereShapeList()
{
	return m_sphereShapeList;
}

inline const b3List2<b3SoftBodySphereShape>& b3SoftBody::GetSphereShapeList() const
{
	return m_sphereShapeList;
}

inline b3List2<b3SoftBodyWorldShape>& b3SoftBody::GetWorldShapeList()
{
	return m_worldShapeList;
}

inline const b3List2<b3SoftBodyWorldShape>& b3SoftBody::GetWorldShapeList() const
{
	return m_worldShapeList;
}

inline const b3List2<b3SoftBodyAnchor>& b3SoftBody::GetAnchorList() const
{
	return m_anchorList;
}

inline b3List2<b3SoftBodyAnchor>& b3SoftBody::GetAnchorList()
{
	return m_anchorList;
}

inline const b3SoftBodyMesh* b3SoftBody::GetMesh() const
{
	return m_mesh;
}

#endif