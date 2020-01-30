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

#include <bounce/softbody/softbody_element.h>

// Compute the elasticity matrix given Young modulus and Poisson's ratio
// This is a 6 x 6 matrix 
static B3_FORCE_INLINE void b3ComputeD(scalar out[36],
	scalar E, scalar nu)
{
	scalar lambda = (nu * E) / ((scalar(1) + nu) * (scalar(1) - scalar(2) * nu));
	scalar mu = E / (scalar(2) * (scalar(1) + nu));

	scalar D[36] =
	{
		lambda + 2 * mu, lambda,          lambda,          0,  0,  0,
		lambda,          lambda + 2 * mu, lambda,          0,  0,  0,
		lambda,          lambda,          lambda + 2 * mu, 0,  0,  0,
		0, 				 0, 			  0, 			   mu, 0,  0,
		0, 				 0, 			  0, 			   0,  mu, 0,
		0, 			 	 0, 			  0, 			   0,  0,  mu
	};

	for (u32 i = 0; i < 36; ++i)
	{
		out[i] = D[i];
	}
}

// Compute B = S * N,
// where S is the operational matrix and N are the shape functions
// This is a 6 x 12 matrix
// A derivation and corresponding simplification for this matrix 
// can be found here:
// https://github.com/erleben/OpenTissue/blob/master/OpenTissue/dynamics/fem/fem_compute_b.h
static B3_FORCE_INLINE void b3ComputeB(scalar out[72],
	const b3Mat33& invE)
{
	// cofactor = det(E)^-1 * cofactor(E)
	b3Mat33 cofactor = b3Transpose(invE);

	// minor = det(E)^-1 * minor(E)
	b3Mat33 minor;

	minor.x.x = cofactor.x.x;
	minor.x.y = -cofactor.x.y;
	minor.x.z = cofactor.x.z;

	minor.y.x = -cofactor.y.x;
	minor.y.y = cofactor.y.y;
	minor.y.z = -cofactor.y.z;

	minor.z.x = cofactor.z.x;
	minor.z.y = -cofactor.z.y;
	minor.z.z = cofactor.z.z;

	scalar e11 = -minor.x.x; 
	scalar e12 = minor.y.x;  
	scalar e13 = -minor.z.x; 

	scalar e21 = minor.x.y; 
	scalar e22 = -minor.y.y; 
	scalar e23 = minor.z.y; 

	scalar e31 = -minor.x.z; 
	scalar e32 = minor.y.z;  
	scalar e33 = -minor.z.z; 

	scalar b1 = -e11 - e12 - e13;
	scalar c1 = -e21 - e22 - e23;
	scalar d1 = -e31 - e32 - e33;

	scalar b2 = e11;
	scalar c2 = e21;
	scalar d2 = e31;

	scalar b3 = e12;
	scalar c3 = e22;
	scalar d3 = e32;

	scalar b4 = e13;
	scalar c4 = e23;
	scalar d4 = e33;

	scalar B[72] =
	{
		b1, 0, 0, c1, d1, 0,
		0, c1, 0, b1, 0, d1,
		0, 0, d1, 0, b1, c1,

		b2, 0, 0, c2, d2, 0,
		0, c2, 0, b2, 0, d2,
		0, 0, d2, 0, b2, c2,

		b3, 0, 0, c3, d3, 0,
		0, c3, 0, b3, 0, d3,
		0, 0, d3, 0, b3, c3,

		b4, 0, 0, c4, d4, 0,
		0, c4, 0, b4, 0, d4,
		0, 0, d4, 0, b4, c4,
	};

	for (u32 i = 0; i < 72; ++i)
	{
		out[i] = B[i];
	}
}

// Return the element in a block matrix given the indices 
// of the element in its corresponding expanded matrix.
static B3_FORCE_INLINE scalar& b3GetElement(b3Mat33 K[16], u32 i, u32 j)
{
	B3_ASSERT(i < 3 * 4);
	B3_ASSERT(j < 3 * 4);

	u32 i0 = i / 3;
	u32 j0 = j / 3;

	b3Mat33& a = K[i0 + 4 * j0];

	u32 ii = i - 3 * i0;
	u32 jj = j - 3 * j0;

	return a(ii, jj);
}

static B3_FORCE_INLINE void b3SetK(b3Mat33 K[16], scalar Ke[144])
{
	for (u32 i = 0; i < 12; ++i)
	{
		for (u32 j = 0; j < 12; ++j)
		{
			scalar k1 = Ke[i + 12 * j];
			scalar& k2 = b3GetElement(K, i, j);

			k2 = k1;
		}
	}
}

void b3SoftBodyElement::ComputeMatrices()
{
	// 6 x 6
	scalar D[36];
	b3ComputeD(D, m_E, m_nu);

	// 6 x 12
	scalar* B = m_B;
	b3ComputeB(B, m_invE);

	// 12 x 6
	scalar BT[72];
	b3Transpose(BT, B, 6, 12);

	// 12 x 6
	scalar BT_D[72];
	b3Mul(BT_D, BT, 12, 6, D, 6, 6);

	// 12 x 12
	scalar BT_D_B[144];
	b3Mul(BT_D_B, BT_D, 12, 6, B, 6, 12);
	for (u32 i = 0; i < 144; ++i)
	{
		BT_D_B[i] *= m_V;
	}

	b3SetK(m_K, BT_D_B);

	// 12 x 6
	scalar* P = m_P;
	b3Mul(P, BT, 12, 6, D, 6, 6);
	for (u32 i = 0; i < 72; ++i)
	{
		P[i] *= m_V;
	}
}