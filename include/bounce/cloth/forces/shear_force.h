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

#ifndef B3_SHEAR_FORCE_H
#define B3_SHEAR_FORCE_H

#include <bounce/cloth/forces/force.h>

class b3ClothTriangle;

struct b3ShearForceDef : public b3ForceDef
{
	b3ShearForceDef()
	{
		type = e_shearForce;
	}

	// Triangle
	b3ClothTriangle* triangle;

	// Shearing stiffness
	float32 shearing;

	// Damping stiffness
	float32 damping;
};

// Shear force acting on a cloth triangle.
class b3ShearForce : public b3Force
{
public:
	bool HasParticle(const b3Particle* particle) const;

	b3ClothTriangle* GetTriangle() const;

	float32 GetShearingStiffness() const;

	float32 GetDampingStiffness() const;

	b3Vec3 GetActionForce1() const;

	b3Vec3 GetActionForce2() const;

	b3Vec3 GetActionForce3() const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3ShearForce(const b3ShearForceDef* def);
	~b3ShearForce();

	void Apply(const b3ClothForceSolverData* data);

	// Solver shared

	// Triangle
	b3ClothTriangle* m_triangle;

	// Shearing stiffness
	float32 m_ks;

	// Damping stiffness
	float32 m_kd;

	// Action forces
	b3Vec3 m_f1, m_f2, m_f3;
};

inline b3ClothTriangle* b3ShearForce::GetTriangle() const
{
	return m_triangle;
}

inline float32 b3ShearForce::GetShearingStiffness() const
{
	return m_ks;
}

inline float32 b3ShearForce::GetDampingStiffness() const
{
	return m_kd;
}

inline b3Vec3 b3ShearForce::GetActionForce1() const
{
	return m_f1;
}

inline b3Vec3 b3ShearForce::GetActionForce2() const
{
	return m_f2;
}

inline b3Vec3 b3ShearForce::GetActionForce3() const
{
	return m_f3;
}

#endif