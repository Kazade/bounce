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

#ifndef B3_SPRING_CLOTH_H
#define B3_SPRING_CLOTH_H

#include <bounce/common/math/mat33.h>
#include <bounce/common/template/array.h>

// Maximum number of shapes per cloth.
#define B3_CLOTH_SHAPE_CAPACITY 32

class b3StackAllocator;

class b3Shape;

struct b3Mesh;

struct b3SpringClothDef
{
	b3SpringClothDef()
	{
		allocator = nullptr;
		mesh = nullptr;
		density = 0.0f;
		ks = 0.0f;
		kb = 0.0f;
		kd = 0.0f;
		r = 0.05f;
		gravity.SetZero();
	}

	// Stack allocator
	b3StackAllocator* allocator;

	// Cloth mesh	
	b3Mesh* mesh;

	// Cloth density in kg/m^3
	float32 density;

	// Streching stiffness
	float32 ks;

	// Bending stiffness
	float32 kb;

	// Damping stiffness
	float32 kd;
	
	// Mass radius
	// Typically this is value is small and is intended for correcting visual artifacts when 
	// the cloth is colliding against a solid.
	float32 r;
	
	// Acceleration due to gravity (m/s^2)
	b3Vec3 gravity;
};

enum b3SpringType
{
	e_strechSpring,
	e_bendSpring
};

struct b3Spring
{
	// Spring type
	b3SpringType type;

	// Mass 1
	u32 i1;

	// Mass 2
	u32 i2;

	// Rest length
	float32 L0;

	// Structural stiffness
	float32 ks;
	
	// Damping stiffness
	float32 kd;
};

// Static masses have zero mass and velocity, and therefore they can't move.
// Kinematic masses are't moved by external and internal forces but can be moved by contact forces.
// Dynamic masses have non-zero mass and can move due to internal and external forces.
enum class b3MassType : u32
{
	e_staticMass,
	e_kinematicMass,
	e_dynamicMass
};

// 
struct b3MassContact
{
	u32 j;
	b3Vec3 n, t1, t2;
	float32 Fn, Ft1, Ft2;
	bool lockN, lockT1, lockT2;
};

// Time step statistics
struct b3SpringClothStep
{
	u32 iterations;
};

// A cloth it treats cloth as a collection of masses connected by springs. 
// Large time steps can be taken.
// If accuracy and stability are required, not performance, 
// you can use this class instead of using b3Cloth.
class b3SpringCloth
{
public:
	b3SpringCloth();
	~b3SpringCloth();

	// Initialize this cloth from a definition.
	void Initialize(const b3SpringClothDef& def);

	// Return the cloth mesh used to initialize this cloth.
	b3Mesh* GetMesh() const;

	// Set the gravitational acceleration applied to this cloth.
	// Units are m/s^2.
	void SetGravity(const b3Vec3& gravity);

	// Return the number of masses in this cloth.
	u32 GetMassCount() const;

	// Return the gravitational acceleration applied to this cloth.
	const b3Vec3& GetGravity() const;

	// Set the type of a given point mass.
	void SetType(u32 i, b3MassType type);

	// Return the type of a given point mass.
	b3MassType GetType(u32 i) const;

	// Set the position of a given point mass.
	// This function will have effect on the position of the point mass 
	// after performing a time step.
	void SetPosition(u32 i, const b3Vec3& position);

	// Return the position of a given point mass.
	const b3Vec3& GetPosition(u32 i) const;

	// Set the velocity of a given point mass.
	void SetVelocity(u32 i, const b3Vec3& velocity);

	// Return the velocity of a given point mass.
	const b3Vec3& GetVelocity(u32 i) const;
	
	// Apply a force to a given point mass.
	void ApplyForce(u32 i, const b3Vec3& force);

	// Return the kinetic (or dynamic) energy in this system.
	float32 GetEnergy() const;
	
	// Return the tension forces (due to springs) acting at each point mass.
	// Units are kg * m / s^2
	void GetTension(b3Array<b3Vec3>& tensions) const;

	// Add a shape to the list of shapes in this cloth. 
	// The cloth will be able to respond to collisions with each shape in the list of shapes.
	void AddShape(b3Shape* shape);

	// Return the number of shapes added to this cloth.
	u32 GetShapeCount() const;

	// Return the list of shapes added to this cloth.
	b3Shape** GetShapes();

	// Return the statistics of the last time step.
	const b3SpringClothStep& GetStep() const;

	// Perform a time step (marches time forward).
	void Step(float32 dt);

	// Set the positions of the mesh vertices to the positions of their associated point masses.
	void Apply() const;

	// Debug draw the cloth mesh.
	void Draw() const;
protected:
	friend class b3SpringSolver;
	
	// Update contacts. 
	// This is where some contacts might be initiated or terminated.
	void UpdateContacts();

	b3StackAllocator* m_allocator;

	b3Mesh* m_mesh;
	float32 m_r;

	b3Vec3 m_gravity;

	b3Vec3* m_x;
	b3Vec3* m_v;
	b3Vec3* m_f;
	float32* m_m;
	float32* m_inv_m;
	b3Vec3* m_y;
	b3Vec3* m_z;
	b3Vec3* m_x0;
	b3MassType* m_types;
	u32 m_massCount;

	b3MassContact* m_contacts;
	
	b3Spring* m_springs;
	u32 m_springCount;

	b3Shape* m_shapes[B3_CLOTH_SHAPE_CAPACITY];
	u32 m_shapeCount;

	b3SpringClothStep m_step;
};

inline b3Mesh* b3SpringCloth::GetMesh() const
{
	return m_mesh;
}

inline const b3Vec3& b3SpringCloth::GetGravity() const
{
	return m_gravity;
}

inline void b3SpringCloth::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline u32 b3SpringCloth::GetMassCount() const
{
	return m_massCount;
}

inline b3MassType b3SpringCloth::GetType(u32 i) const
{
	B3_ASSERT(i < m_massCount);
	return m_types[i];
}

inline void b3SpringCloth::SetType(u32 i, b3MassType type)
{
	B3_ASSERT(i < m_massCount);
	
	if (m_types[i] == type)
	{
		return;
	}

	m_types[i] = type;
	
	m_f[i].SetZero();
	
	if (type == b3MassType::e_staticMass)
	{
		m_v[i].SetZero();
		m_y[i].SetZero();
		
		m_contacts[i].lockN = false;
	}
}

inline void b3SpringCloth::SetPosition(u32 i, const b3Vec3& position)
{
	B3_ASSERT(i < m_massCount);
	m_y[i] += position - m_x[i];
}

inline const b3Vec3& b3SpringCloth::GetPosition(u32 i) const
{
	B3_ASSERT(i < m_massCount);
	return m_x[i];
}

inline void b3SpringCloth::SetVelocity(u32 i, const b3Vec3& velocity)
{
	B3_ASSERT(i < m_massCount);
	
	if (m_types[i] == b3MassType::e_staticMass)
	{
		return;
	}

	m_v[i] = velocity;
}

inline const b3Vec3& b3SpringCloth::GetVelocity(u32 i) const
{
	B3_ASSERT(i < m_massCount);
	return m_v[i];
}

inline void b3SpringCloth::ApplyForce(u32 i, const b3Vec3& force)
{
	B3_ASSERT(i < m_massCount);
	
	if (m_types[i] != b3MassType::e_dynamicMass)
	{
		return;
	}

	m_f[i] += force;
}

inline float32 b3SpringCloth::GetEnergy() const
{
	float32 E = 0.0f;
	for (u32 i = 0; i < m_massCount; ++i)
	{
		E += m_m[i] * b3Dot(m_v[i], m_v[i]);
	}
	return 0.5f * E;
}

inline u32 b3SpringCloth::GetShapeCount() const
{
	return m_shapeCount;
}

inline b3Shape** b3SpringCloth::GetShapes() 
{
	return m_shapes;
}

inline const b3SpringClothStep& b3SpringCloth::GetStep() const
{
	return m_step;
}

#endif