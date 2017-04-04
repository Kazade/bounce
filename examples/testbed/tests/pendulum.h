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

#ifndef PENDULUM_H
#define PENDULUM_H

extern Settings g_settings;
extern DebugDraw* g_debugDraw;

class Pendulum : public Test
{
public:
	Pendulum()
	{
		m_g = -10.0f;

		m_r = 10.0f;
		m_m = 1.0f;
		m_I = m_m * m_r * m_r;
		
		// Initial state
		m_theta = 0.5f * B3_PI;
		m_omega = 0.0f;
	}

	void Step()
	{
		float32 h = g_settings.hertz > 0.0f ? 1.0f / g_settings.hertz : 0.0f;
		
		if (g_settings.pause)
		{
			if (g_settings.singleStep)
			{
				g_settings.singleStep = false;
			}
			else
			{
				h = 0.0f;
			}
		}

		// Solution (acceleration)
		float32 omega_dot = -m_g / m_r * sin(m_theta);

		// Integrate acceleration
		m_omega += h * omega_dot;

		// Integrate velocity
		m_theta += h * m_omega;

		// Convert from polar coordinates (r, theta) to Cartesian coordinates (x, y)
		b3Vec3 c;
		c.x = m_r * sin(m_theta);
		c.y = m_r * cos(m_theta);
		c.z = 0.0f;
		g_debugDraw->DrawSolidSphere(c, 1.0f, b3Color_white);

		b3Vec3 pole;
		pole.SetZero();
		g_debugDraw->DrawSegment(pole, c, b3Color_white);

		// Kinetic energy
		float32 T = 0.5f * m_I * m_omega * m_omega;

		// Potential energy
		float32 V = -m_m * m_g * m_r * cos(m_theta);

		// Lagrangian
		float32 L = T - V;

		static char s[256];
		sprintf(s, "T = %f \nV = %f \nL = %f", T, V, L);
		g_debugDraw->DrawString(s, b3Color_white);
	}

	static Test* Create()
	{
		return new Pendulum();
	}

	// Gravity
	float32 m_g;
	
	// Mass, inertia
	float32 m_m, m_I;

	// Radial coordinate
	float32 m_r;

	// The allowable generalized coordinate in polar coordinate frame.
	// Only motions satisfying the constraints can be described 
	// in this frame. Therefore, all solutions satisfy the constraints.
	// This is the so called reduced coordinates approach.
	float32 m_theta;  
	
	// Velocity
	float32 m_omega; 
};

#endif