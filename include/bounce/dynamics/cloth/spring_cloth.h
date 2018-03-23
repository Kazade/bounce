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

class b3StackAllocator;
class b3Draw;

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
		gravity.SetZero();
	}

	// Stack allocator
	b3StackAllocator* allocator;

	// Cloth mesh	
	b3Mesh* mesh;

	// Cloth density in kg/m^2
	float32 density;

	// Streching stiffness
	float32 ks;

	// Bending stiffness
	float32 kb;

	// Damping stiffness
	float32 kd;
	
	// Force due to gravity 
	b3Vec3 gravity;
};

struct b3Spring
{
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
// Dynamic masses have non-zero mass and can move due to internal and external forces.
enum b3MassType
{
	e_staticMass,
	e_dynamicMass
};

// Time step statistics
struct b3SpringClothStep
{
	u32 iterations;
};

// This class implements a cloth. It treats cloth as a collection 
// of masses connected by springs. 
// Large time steps can be taken.
// If accuracy and stability are required, not performance, 
// you can use this class instead of using b3Cloth.
class b3SpringCloth
{
public:
	b3SpringCloth();
	~b3SpringCloth();

	//
	void Initialize(const b3SpringClothDef& def);

	//
	void SetGravity(const b3Vec3& gravity);

	//
	const b3Vec3& GetGravity() const;

	//
	void SetType(u32 i, b3MassType type);

	//
	b3MassType GetType(u32 i) const;

	// 
	const b3SpringClothStep& GetStep() const;

	//
	void Step(float32 dt);

	//
	void Apply() const;

	//
	void Draw(b3Draw* draw) const;
protected:
	b3StackAllocator* m_allocator;

	b3Mesh* m_mesh;

	b3Vec3 m_gravity;

	b3Vec3* m_x;
	b3Vec3* m_v;
	b3Vec3* m_f;
	float32* m_inv_m;
	b3MassType* m_massTypes;
	u32 m_massCount;

	b3Spring* m_springs;
	u32 m_springCount;

	b3SpringClothStep m_step;
};

inline const b3Vec3& b3SpringCloth::GetGravity() const
{
	return m_gravity;
}

inline void b3SpringCloth::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3MassType b3SpringCloth::GetType(u32 i) const
{
	B3_ASSERT(i < m_massCount);
	return m_massTypes[i];
}

inline void b3SpringCloth::SetType(u32 i, b3MassType type)
{
	B3_ASSERT(i < m_massCount);
	m_massTypes[i] = type;
}

inline const b3SpringClothStep& b3SpringCloth::GetStep() const
{
	return m_step;
}

#endif