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

#ifndef B3_BEND_FORCE_H
#define B3_BEND_FORCE_H

#include <bounce/dynamics/cloth/force.h>

struct b3BendForceDef : public b3ForceDef
{
	b3BendForceDef()
	{
		type = e_bendForce;
		p1 = nullptr;
		p2 = nullptr;
		p3 = nullptr;
		p4 = nullptr;
		restDistance = 0.0f;
		restAngle = 0.0f;
		structural = 0.0f;
		damping = 0.0f;
	}

	// 
	void Initialize(b3Particle* particle1, b3Particle* particle2, b3Particle* particle3, b3Particle* particle4, 
		float32 structuralStiffness, float32 dampingStiffness);

	// Particle 1
	b3Particle* p1;

	// Particle 2
	b3Particle* p2;

	// Particle 3
	b3Particle* p3;
	
	// Particle 4
	b3Particle* p4;
	
	// Rest distance
	float32 restDistance;
	
	// Rest angle
	float32 restAngle;

	// Structural stiffness
	float32 structural;

	// Damping stiffness
	float32 damping;
};

// 
class b3BendForce : public b3Force
{
public:
	b3Particle* GetParticle1();

	b3Particle* GetParticle2();

	b3Particle* GetParticle3();

	b3Particle* GetParticle4();
	
	float32 GetRestDistance() const;
	
	float32 GetRestAngle() const;

	float32 GetStructuralStiffness() const;

	float32 GetDampingStiffness() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3BendForce(const b3BendForceDef* def);
	~b3BendForce();

	void Apply(const b3ClothSolverData* data);

	// Solver shared

	// Particle 1
	b3Particle* m_p1;

	// Particle 2
	b3Particle* m_p2;

	// Particle 3
	b3Particle* m_p3;
	
	// Particle 4
	b3Particle* m_p4;

	// Rest distance
	float32 m_L0;

	// Rest angle
	float32 m_angle0;
	
	// Structural stiffness
	float32 m_ks;

	// Structural stiffness
	float32 m_kd;
};

inline b3Particle* b3BendForce::GetParticle1()
{
	return m_p1;
}

inline b3Particle* b3BendForce::GetParticle2()
{
	return m_p2;
}

inline b3Particle* b3BendForce::GetParticle3()
{
	return m_p3;
}

inline b3Particle* b3BendForce::GetParticle4()
{
	return m_p4;
}

inline float32 b3BendForce::GetRestAngle() const
{
	return m_angle0;
}

inline float32 b3BendForce::GetStructuralStiffness() const
{
	return m_ks;
}

inline float32 b3BendForce::GetDampingStiffness() const
{
	return m_kd;
}

#endif