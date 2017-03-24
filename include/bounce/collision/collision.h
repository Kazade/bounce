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

#ifndef B3_COLLISION_H
#define B3_COLLISION_H

#include <bounce/common/geometry.h>
#include <bounce/collision/shapes/aabb3.h>

// Input for a ray cast.
struct b3RayCastInput
{
	b3Vec3 p1; // first point on segment
	b3Vec3 p2; // second point on segment
	float32 maxFraction; // maximum intersection
};

// Output of a ray cast.
struct b3RayCastOutput
{
	float32 fraction; // time of intersection on ray-segment
	b3Vec3 normal; // surface normal of intersection
};

#endif