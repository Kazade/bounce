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

#ifndef B3_SPRING_FORCE_H
#define B3_SPRING_FORCE_H

#include <bounce/cloth/force.h>

struct b3SpringForceDef : public b3ForceDef
{
	b3SpringForceDef()
	{
		type = e_springForce;
		p1 = nullptr;
		p2 = nullptr;
		restLength = 0.0f;
		structural = 0.0f;
		damping = 0.0f;
	}

	// 
	void Initialize(b3Particle* particle1, b3Particle* particle2, float32 structuralStiffness, float32 dampingStiffness);

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Rest length
	float32 restLength;

	// Structural stiffness
	float32 structural;

	// Damping stiffness
	float32 damping;
};

// 
class b3SpringForce : public b3Force
{
public:
	b3Particle* GetParticle1();

	b3Particle* GetParticle2();

	float32 GetRestLenght() const;

	float32 GetStructuralStiffness() const;

	float32 GetDampingStiffness() const;

	b3Vec3 GetActionForce() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3SpringForce(const b3SpringForceDef* def);
	~b3SpringForce();

	void Apply(const b3ClothSolverData* data);

	// Solver shared

	// Particle 1
	b3Particle* m_p1;

	// Particle 2
	b3Particle* m_p2;

	// Rest length
	float32 m_L0;

	// Structural stiffness
	float32 m_ks;

	// Damping stiffness
	float32 m_kd;

	// Applied internal force (on particle 1)
	b3Vec3 m_f;
};

inline b3Particle * b3SpringForce::GetParticle1()
{
	return m_p1;
}

inline b3Particle* b3SpringForce::GetParticle2()
{
	return m_p2;
}

inline float32 b3SpringForce::GetRestLenght() const
{
	return m_L0;
}

inline float32 b3SpringForce::GetStructuralStiffness() const
{
	return m_ks;
}

inline float32 b3SpringForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3SpringForce::GetActionForce() const
{
	return m_f;
}

#endif