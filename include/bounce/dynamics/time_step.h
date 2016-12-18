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

#ifndef B3_TIME_STEP_H
#define B3_TIME_STEP_H

#include <bounce\common\math\vec3.h>
#include <bounce\common\math\mat33.h>
#include <bounce\common\math\quat.h>

struct b3Position
{
	b3Vec3 x;
	b3Quat q;
};

struct b3Velocity
{
	b3Vec3 v;
	b3Vec3 w;
};

struct b3SolverData
{
	b3Position* positions;
	b3Velocity* velocities;
	float32 dt;
	float32 invdt;
};

enum b3LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

// Compute the time derivative of an orientation given
// the angular velocity of the rotating frame represented by the orientation.
inline b3Quat b3Derivative(const b3Quat& orientation, const b3Vec3& velocity)
{
	b3Quat xf(0.5f * velocity.x, 0.5f * velocity.y, 0.5f * velocity.z, 0.0f);
	return xf * orientation;
}

// Integrate an orientation over a time step given
// the current orientation, angular velocity of the rotating frame
// represented by the orientation, and the time step dt.
inline b3Quat b3Integrate(const b3Quat& orientation, const b3Vec3& velocity, float32 dt)
{
	// Integrate from [t0, t0 + h] using the explicit Euler method
	b3Quat qdot = b3Derivative(orientation, velocity);
	b3Quat integral = dt * qdot;
	return orientation + integral;
}

// Compute the time derivative of an orientation given
// the angular velocity of the rotating frame represented by the orientation.
inline b3Mat33 b3Derivative(const b3Mat33& orientation, const b3Vec3& velocity)
{
	// Rate of change of a basis in a rotating frame:
	// xdot = cross(w, ex)
	// ydot = cross(w, ey)
	// zdot = cross(w, ez)
	// This should yield in:
	// qdot = skew(velocity) * q
	b3Mat33 xf = b3Skew(velocity);
	return xf * orientation;
}

// Integrate an orientation over a time step given
// the current orientation, angular velocity of the rotating frame
// represented by the orientation, and the time step dt.
inline b3Mat33 b3Integrate(const b3Mat33& orientation, const b3Vec3& velocity, float32 dt)
{
	b3Mat33 qdot = b3Derivative(orientation, velocity);
	b3Mat33 integral = dt * qdot;
	return orientation + integral;
}

// These structures contain the elapsed time it took 
// to run particular method.
struct b3CollideProfile
{
	float64 broadphase;
	float64 narrowphase;
};

struct b3SolverProfile
{
	float64 initializeContacts;
	float64 initializeJoints;
	float64 solveVelocity;
	float64 solvePosition;
};

struct b3Profile
{
	float64 total;
	b3CollideProfile collide;
	b3SolverProfile solver;
};

#endif
