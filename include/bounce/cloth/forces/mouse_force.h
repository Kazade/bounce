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

class b3ClothTriangle;

struct b3MouseForceDef : public b3ForceDef
{
	b3MouseForceDef()
	{
		type = e_mouseForce;
	}

	// Particle
	b3Particle* particle;

	// Triangle
	b3ClothTriangle* triangle;

	// Barycentric coordinates on triangle
	float32 w2, w3, w4;

	// Mouse stiffness
	float32 mouse;

	// Damping stiffness
	float32 damping;
};

// Mouse force acting on a particle and triangle.
class b3MouseForce : public b3Force
{
public:
	bool HasParticle(const b3Particle* particle) const;

	b3Particle* GetParticle() const;
	
	b3ClothTriangle* GetTriangle() const;

	float32 GetMouseStiffness() const;

	float32 GetDampingStiffness() const;

	b3Vec3 GetActionForce1() const;

	b3Vec3 GetActionForce2() const;

	b3Vec3 GetActionForce3() const;

	b3Vec3 GetActionForce4() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3MouseForce(const b3MouseForceDef* def);
	~b3MouseForce();

	void Apply(const b3ClothForceSolverData* data);

	// Solver shared

	// Particle
	b3Particle* m_particle;
	
	// Triangle
	b3ClothTriangle* m_triangle;

	// Barycentric coordinates
	float32 m_w2, m_w3, m_w4;

	// Mouse stiffness
	float32 m_km;

	// Damping stiffness
	float32 m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3, m_f4;
};

inline b3Particle* b3MouseForce::GetParticle() const
{
	return m_particle;
}

inline b3ClothTriangle* b3MouseForce::GetTriangle() const
{
	return m_triangle;
}

inline float32 b3MouseForce::GetMouseStiffness() const
{
	return m_km;
}

inline float32 b3MouseForce::GetDampingStiffness() const
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