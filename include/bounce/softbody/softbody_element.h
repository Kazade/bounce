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

#ifndef B3_SOFTBODY_ELEMENT_H
#define B3_SOFTBODY_ELEMENT_H

#include <bounce/common/math/quat.h>

class b3SoftBody;

// Soft body tetrahedron element
class b3SoftBodyElement
{
public:
	// Set the material Young Modulus of elasticity in [0, inf].
	void SetE(scalar E);
	
	// Get the material Young Modulus of elasticity in [0, inf].
	scalar GetE() const;

	// Set the Poisson's ratio in [0, 0.5].
	void SetNU(scalar nu);

	// Get the Poisson's ratio in [0, 0.5].
	scalar GetNU() const;

	// Set the elastic strain yield in range [0, inf].
	// Set this value to inf to disable plasticity.
	void SetCYield(scalar yield);

	// Get the elastic strain yield in range [0, inf].
	scalar GetCYield() const;

	// Set the material creep rate in hertz.
	void SetCCreep(scalar hz);

	// Get the material creep rate in hertz.
	scalar GetCCreep() const;

	// Set the material maximum plastic strain in the range [0, inf].
	void SetCMax(scalar max);

	// Get the material maximum plastic strain in the range [0, inf].
	scalar GetCMax() const;
private:
	friend class b3SoftBody;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyForceSolver;

	b3SoftBodyElement() { }

	~b3SoftBodyElement() { }

	void ComputeMatrices();

	// Reference volume
	scalar m_V;

	// Elasticity
	scalar m_E;
	scalar m_nu;

	// Plasticity
	scalar m_c_yield;
	scalar m_c_creep;
	scalar m_c_max;
	scalar m_epsilon_plastic[6]; // 6 x 1

	// Solver shared
	b3Mat33 m_invE; // 3 x 3
	b3Quat m_q; // 3 x 3
	b3Mat33* m_Kp[16]; // 12 x 12
	b3Mat33 m_K[16]; // 12 x 12
	scalar m_B[72]; // 6 x 12
	scalar m_P[72]; // V * BT * E -> 12 x 6

	// Soft body
	b3SoftBody* m_body;
};

inline void b3SoftBodyElement::SetE(scalar E)
{
	B3_ASSERT(E > scalar(0));
	if (E != m_E)
	{
		m_E = E;
		ComputeMatrices();
	}
}

inline scalar b3SoftBodyElement::GetE() const
{
	return m_E;
}

inline void b3SoftBodyElement::SetNU(scalar nu)
{
	B3_ASSERT(nu >= scalar(0) && nu <= scalar(0.5));
	if (nu != m_nu)
	{
		m_nu = nu;
		ComputeMatrices();
	}
}

inline scalar b3SoftBodyElement::GetNU() const
{
	return m_nu;
}

inline void b3SoftBodyElement::SetCYield(scalar yield)
{
	B3_ASSERT(yield >= scalar(0));
	m_c_yield = yield;
}

inline scalar b3SoftBodyElement::GetCYield() const
{
	return m_c_yield;
}

inline void b3SoftBodyElement::SetCCreep(scalar hz)
{
	B3_ASSERT(hz >= scalar(0));
	m_c_creep = hz;
}

inline scalar b3SoftBodyElement::GetCCreep() const
{
	return m_c_creep;
}

inline void b3SoftBodyElement::SetCMax(scalar max)
{
	B3_ASSERT(max >= scalar(0));
	m_c_max = max;
}

inline scalar b3SoftBodyElement::GetCMax() const
{
	return m_c_max;
}

#endif