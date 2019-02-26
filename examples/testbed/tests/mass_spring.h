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

#ifndef MASS_SPRING_H
#define MASS_SPRING_H

class MassSpring : public Test
{
public:
	MassSpring()
	{
		m_x.Set(0.0f, 5.0f, 0.0f);

		m_v.SetZero();

		m_k = 100.0f;

		m_iterations = 0;
	}

	void Solve(float32 h)
	{
		// ODE
		// f(Y) = dY / dt = [v] 
		//				    [-k * x]

		// 1. Apply Implicit Euler
		
		// Y(t + h) = Y(t) + h * f( Y(t + h) )
		// G( Y(t + h) ) = Y(t + h) - Y(t) - h * f( Y(t + h) ) = 0
		
		// 2. Solve G = 0
		
		// Newton-Raphson Iteration
		//
		// Y(t + h) = 
		// Y(t + h)_0 - G( Y(t + h)_0 ) / G'( Y(t + h)_0 ) = 
		// Y(t + h)_0 - G'( Y(t + h)_0 )^-1 * ( Y(t + h)_0 - Y(t) - h * f( Y(t + h)_0 )

		// G'( Y ) = I - h * del_f / del_Y  

		// del_f / del_Y = [del_f1 / del_x del_f1 / del_v] = [0        I]
		//                 [del_f2 / del_x del_f2 / del_v]   [-k * I   0]

		// G'( Y ) = [I 0] - [0           h * I] =  [I            -h * I]         
		//	         [0 I]   [-h * k * I      0]    [h * k * I         I]  

		// Compute Jacobian
		b3Mat33 I = b3Mat33_identity;

		b3Mat33 A, B, C, D;

		A = I;
		B = -h * I;
		C = h * m_k * I;
		D = I;

		// Invert 
		// Block matrix inversion
		b3Mat33 invD = b3Inverse(D);
		b3Mat33 B_invD = B * invD;

		b3Mat33 invJ_A = b3Inverse(A - B_invD * C);
		b3Mat33 invJ_B = -invJ_A * B_invD;
		b3Mat33 invJ_C = -invD * C * invJ_A;
		b3Mat33 invJ_D = invD + invD * C * invJ_A * B_invD;

		// Initial guess 
		b3Vec3 f1 = m_v;
		b3Vec3 f2 = -m_k * m_x;

		b3Vec3 Y1 = m_x + h * f1;
		b3Vec3 Y2 = m_v + h * f2;
		
		const float32 kTol = 0.05f;

		const u32 kMaxIterations = 20;
		
		float32 eps0 = 0.0f;

		float32 eps1 = B3_MAX_FLOAT;

		m_iterations = 0;

		while (m_iterations < kMaxIterations && eps1 > kTol * kTol * eps0)
		{	
			// Evaluate f(Y_n-1)
			f1 = Y2;
			f2 = -m_k * Y1;
			
			// Residual vector 
			b3Vec3 G1 = Y1 - m_x - h * f1;
			b3Vec3 G2 = Y2 - m_v - h * f2;

			eps1 = b3Dot(G1, G1) + b3Dot(G2, G2);

			// Solve Ax = b 
			b3Vec3 x1 = invJ_A * G1 + invJ_B * G2;
			b3Vec3 x2 = invJ_C * G1 + invJ_D * G2;

			Y1 -= x1;
			Y2 -= x2;

			++m_iterations;
		}

		// Update state
		m_x = Y1;
		m_v = Y2;
	}

	void Step()
	{
		float32 h = g_testSettings->inv_hertz;

		Solve(h);

		g_draw->DrawSolidSphere(m_x, 0.25f, b3Color_white);
		
		g_draw->DrawSegment(b3Vec3_zero, m_x, b3Color_white);

		g_draw->DrawString(b3Color_white, "Iterations = %u", m_iterations);

		float32 E = 0.5f * b3Dot(m_v, m_v);
		g_draw->DrawString(b3Color_white, "E = %f", E);
	}

	static Test* Create()
	{
		return new MassSpring();
	}

	// State
	b3Vec3 m_x, m_v;

	// Stiffness
	float32 m_k;

	//
	u32 m_iterations;
};

#endif