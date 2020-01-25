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

#ifndef B3_STRETCH_FORCE_H
#define B3_STRETCH_FORCE_H

#include <bounce/cloth/forces/force.h>

// Stretch force definition.
// This requires defining the (u, v) coordinates 
// of the triangle and some parameters.
struct b3StretchForceDef : public b3ForceDef
{
	b3StretchForceDef()
	{
		type = e_stretchForce;
		u1 = scalar(0);
		v1 = scalar(0);
		u2 = scalar(0);
		v2 = scalar(0);
		u3 = scalar(0);
		v3 = scalar(0);
		alpha = scalar(0);
		stretching_u = scalar(0);
		damping_u = scalar(0);
		b_u = scalar(1);
		stretching_v = scalar(0);
		damping_v = scalar(0);
		b_v = scalar(1);
	}

	// Initialize this definition from rest positions
	void Initialize(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3);

	// Particle 1
	b3ClothParticle* p1;

	// Particle 2
	b3ClothParticle* p2;

	// Particle 3
	b3ClothParticle* p3;

	// (u, v) coordinates for vertex 1 in the rest state
	scalar u1, v1;

	// (u, v) coordinates for vertex 2 in the rest state
	scalar u2, v2;
	
	// (u, v) coordinates for vertex 3 in the rest state
	scalar u3, v3;
	
	// Typically this is the triangle area in the rest state.
	scalar alpha;

	// Stretching stiffness in u direction
	scalar stretching_u;

	// Damping stiffness in u direction
	scalar damping_u;
	
	// Desired strechiness in u direction
	scalar b_u;

	// Stretching stiffness in v direction
	scalar stretching_v;

	// Damping stiffness in v direction
	scalar damping_v;
	
	// Desired strechiness in v direction
	scalar b_v;
};

// Stretch force acting on a cloth triangle.
// This maintains the triangle edge lengths in the (u, v) 
// frame of reference at a given desired normalized rest distance 
// in the direction of the u and v coordinates.
class b3StretchForce : public b3Force
{
public:
	// Has this force a given particle?
	bool HasParticle(const b3ClothParticle* particle) const;

	// Get the particle 1.
	b3ClothParticle* GetParticle1();
	const b3ClothParticle* GetParticle1() const;

	// Get the particle 2.
	b3ClothParticle* GetParticle2();
	const b3ClothParticle* GetParticle2() const;
	
	// Get the particle 2.
	b3ClothParticle* GetParticle3();
	const b3ClothParticle* GetParticle3() const;
	
	// Get the spring stiffness in the u direction.
	scalar GetStrechingStiffnessU() const;

	// Get the damping stiffness in the u direction.
	scalar GetDampingStiffnessU() const;
	
	// Get the normalized rest lenght in the u direction.
	scalar GetBU() const;
	
	// Get the spring stiffness in the v direction.
	scalar GetStrechingStiffnessV() const;

	// Get the damping stiffness in the v direction.
	scalar GetDampingStiffnessV() const;
	
	// Get the normalized rest lenght in the v direction.
	scalar GetBV() const;
	
	// Get the force acting on particle 1.
	b3Vec3 GetActionForce1() const;

	// Get the force acting on particle 2.
	b3Vec3 GetActionForce2() const;
	
	// Get the force acting on particle 3.
	b3Vec3 GetActionForce3() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3StretchForce(const b3StretchForceDef* def);
	~b3StretchForce();

	void Apply(const b3ClothForceSolverData* data);

	// Particle 1
	b3ClothParticle* m_p1;

	// Particle 2
	b3ClothParticle* m_p2;
	
	// Particle 3
	b3ClothParticle* m_p3;

	// Alpha
	scalar m_alpha;

	// (u, v) matrix
	scalar m_du1, m_dv1;
	scalar m_du2, m_dv2;
	scalar m_inv_det;

	// dwudx, dwvdx
	b3Vec3 m_dwudx, m_dwvdx;

	// Streching stiffness in u direction
	scalar m_ks_u;

	// Damping stiffness in u direction
	scalar m_kd_u;
	
	// Desired strechiness in u direction
	scalar m_b_u;

	// Streching stiffness in v direction
	scalar m_ks_v;

	// Damping stiffness in v direction
	scalar m_kd_v;
	
	// Desired strechiness in v direction
	scalar m_b_v;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3;
};

inline b3ClothParticle* b3StretchForce::GetParticle1()
{
	return m_p1;
}

inline const b3ClothParticle* b3StretchForce::GetParticle1() const
{
	return m_p1;
}

inline b3ClothParticle* b3StretchForce::GetParticle2()
{
	return m_p2;
}

inline const b3ClothParticle* b3StretchForce::GetParticle2() const
{
	return m_p2;
}

inline b3ClothParticle* b3StretchForce::GetParticle3()
{
	return m_p3;
}

inline const b3ClothParticle* b3StretchForce::GetParticle3() const
{
	return m_p3;
}

inline scalar b3StretchForce::GetStrechingStiffnessU() const
{
	return m_ks_u;
}

inline scalar b3StretchForce::GetDampingStiffnessU() const
{
	return m_kd_u;
}

inline scalar b3StretchForce::GetBU() const
{
	return m_b_u;
}

inline scalar b3StretchForce::GetStrechingStiffnessV() const
{
	return m_ks_v;
}

inline scalar b3StretchForce::GetDampingStiffnessV() const
{
	return m_kd_v;
}

inline scalar b3StretchForce::GetBV() const
{
	return m_b_v;
}

inline b3Vec3 b3StretchForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3StretchForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3StretchForce::GetActionForce3() const
{
	return m_f3;
}

#endif