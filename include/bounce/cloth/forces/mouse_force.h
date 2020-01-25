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

#ifndef B3_MOUSE_FORCE_H
#define B3_MOUSE_FORCE_H

#include <bounce/cloth/forces/force.h>

// Mouse force definition.
// This requires defining a particle and a triangle 
// and the coordinates of the particle local to the triangle 
// in the rest state using a barycentric coordinate system.
// You must also provide spring parameters.
struct b3MouseForceDef : public b3ForceDef
{
	b3MouseForceDef()
	{
		type = e_mouseForce; 
		w2 = scalar(0);
		w3 = scalar(0);
		w4 = scalar(0);
		restLength = scalar(0);
		mouse = scalar(0);
		damping = scalar(0);
	}

	// Particle 1
	b3ClothParticle* p1;

	// Particle 2
	b3ClothParticle* p2;
	
	// Particle 3
	b3ClothParticle* p3;
	
	// Particle 4
	b3ClothParticle* p4;
	
	// Barycentric coordinates on triangle
	scalar w2, w3, w4;

	// Mouse stiffness
	scalar mouse;

	// Damping stiffness
	scalar damping;

	// Rest length
	scalar restLength;
};

// Mouse force acting on a particle and triangle.
// This force will keep a point on one particle and the other on the 
// triangle to a given desired distance.
class b3MouseForce : public b3Force
{
public:
	// Has this force a given particle?
	bool HasParticle(const b3ClothParticle* particle) const;

	// Get the particle 1.
	const b3ClothParticle* GetParticle1() const;
	b3ClothParticle* GetParticle1();

	// Get the particle 2.
	const b3ClothParticle* GetParticle2() const;
	b3ClothParticle* GetParticle2();
	
	// Get the particle 3.
	const b3ClothParticle* GetParticle3() const;
	b3ClothParticle* GetParticle3();
	
	// Get the particle 4.
	const b3ClothParticle* GetParticle4() const;
	b3ClothParticle* GetParticle4();
	
	// Get the natural spring length.
	scalar GetRestLenght() const;

	// Get the mouse stiffness.
	scalar GetMouseStiffness() const;

	// Get the damping stiffness.
	scalar GetDampingStiffness() const;

	// Get the force acting on particle 1.
	b3Vec3 GetActionForce1() const;

	// Get the force acting on particle 2.
	b3Vec3 GetActionForce2() const;

	// Get the force acting on particle 3.
	b3Vec3 GetActionForce3() const;

	// Get the force acting on particle 4.
	b3Vec3 GetActionForce4() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3MouseForce(const b3MouseForceDef* def);
	~b3MouseForce();

	void Apply(const b3ClothForceSolverData* data);

	// Particle 1
	b3ClothParticle* m_p1;

	// Particle 2
	b3ClothParticle* m_p2;
	
	// Particle 3
	b3ClothParticle* m_p3;
	
	// Particle 4
	b3ClothParticle* m_p4;
	
	// Barycentric coordinates in the rest state
	scalar m_w2, m_w3, m_w4;

	// Mouse stiffness
	scalar m_km;

	// Damping stiffness
	scalar m_kd;

	// Rest length
	scalar m_L0;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3, m_f4;
};

inline const b3ClothParticle* b3MouseForce::GetParticle1() const
{
	return m_p1;
}

inline b3ClothParticle* b3MouseForce::GetParticle1()
{
	return m_p1;
}

inline const b3ClothParticle* b3MouseForce::GetParticle2() const
{
	return m_p2;
}

inline b3ClothParticle* b3MouseForce::GetParticle2()
{
	return m_p2;
}

inline const b3ClothParticle* b3MouseForce::GetParticle3() const
{
	return m_p3;
}

inline b3ClothParticle* b3MouseForce::GetParticle3()
{
	return m_p3;
}

inline const b3ClothParticle* b3MouseForce::GetParticle4() const
{
	return m_p4;
}

inline b3ClothParticle* b3MouseForce::GetParticle4()
{
	return m_p4;
}

inline scalar b3MouseForce::GetRestLenght() const
{
	return m_L0;
}

inline scalar b3MouseForce::GetMouseStiffness() const
{
	return m_km;
}

inline scalar b3MouseForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3MouseForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3MouseForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3MouseForce::GetActionForce3() const
{
	return m_f3;
}

inline b3Vec3 b3MouseForce::GetActionForce4() const
{
	return m_f4;
}

#endif