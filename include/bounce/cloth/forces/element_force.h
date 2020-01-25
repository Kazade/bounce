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

#ifndef B3_ELEMENT_FORCE_H
#define B3_ELEMENT_FORCE_H

#include <bounce/cloth/forces/force.h>
#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

// Element force definition.
// This requires defining the triangle in the rest state and 
// some material parameters.
struct b3ElementForceDef : public b3ForceDef
{
	b3ElementForceDef()
	{
		type = e_elementForce;
		E_x = scalar(0);
		E_y = scalar(0);
		E_s = scalar(0);
		nu_xy = scalar(0);
		nu_yx = scalar(0);
	}

	// Particle 1
	b3ClothParticle* p1;

	// Particle 2
	b3ClothParticle* p2;

	// Particle 3
	b3ClothParticle* p3;

	// Triangle vertices in rest state
	b3Vec3 v1, v2, v3;

	// Young Modulus in x direction
	scalar E_x;

	// Young Modulus in y direction
	scalar E_y;

	// Shear Modulus 
	scalar E_s;

	// x, y Poisson's Ratio 
	scalar nu_xy;

	// y, x Poisson's Ratio
	scalar nu_yx;
};

// Element force acting on a cloth triangle.
class b3ElementForce : public b3Force
{
public:
	// Has this force a given particle?
	bool HasParticle(const b3ClothParticle* particle) const;
private:
	friend class b3Force;
	friend class b3Cloth;

	b3ElementForce(const b3ElementForceDef* def);
	~b3ElementForce();

	void Apply(const b3ClothForceSolverData* data);

	// Particle 1
	b3ClothParticle* m_p1;

	// Particle 2
	b3ClothParticle* m_p2;

	// Particle 3
	b3ClothParticle* m_p3;

	// Rest triangle vertices in 2D
	b3Vec2 m_x1, m_x2, m_x3;

	// Rest triangle area in 2D
	scalar m_A;

	// Initial inverse deformation in 2D
	b3Mat22 m_invS;

	// Young Modulus in 2D
	scalar m_E_x, m_E_y, m_E_s;

	// Poisson Ratio in 2D
	scalar m_nu_xy, m_nu_yx;

	// Elasticity tensor
	// This is a 3x3 matrix
	b3Mat33 m_C;

	// Shape functions (barycentric) matrix
	// This is a 3x6 matrix
	scalar m_B[18];

	// Blocked stiffness matrix
	// This is a 6x6 matrix
	b3Mat22 m_K[9];
};

#endif