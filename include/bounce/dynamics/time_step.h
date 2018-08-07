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

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>

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
	b3Mat33* invInertias;
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

// Return the Steiner's matrix given the displacement vector from the old 
// center of rotation to the new center of rotation.
// The result equals to transpose( skew(v) ) * skew(v) or diagonal(v^2) - outer(v, v)
inline b3Mat33 b3Steiner(const b3Vec3& v)
{
	float32 xx = v.x * v.x;
	float32 yy = v.y * v.y;
	float32 zz = v.z * v.z;

	b3Mat33 S;
	
	S.x.x = yy + zz;
	S.x.y = -v.x * v.y;
	S.x.z = -v.x * v.z;

	S.y.x = S.x.y;
	S.y.y = xx + zz;
	S.y.z = -v.y * v.z;

	S.z.x = S.x.z;
	S.z.y = S.y.z;
	S.z.z = xx + yy;

	return S;
}

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Mat33& rotation)
{
	return rotation * inertia * b3Transpose(rotation);
}

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Quat& rotation)
{
	b3Mat33 R = b3QuatMat33(rotation);
	
	return R * inertia * b3Transpose(R);
}

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

#endif
