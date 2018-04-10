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

#ifndef B3_CAPSULE_H
#define B3_CAPSULE_H

#include <bounce/common/math/vec3.h>

struct b3Capsule
{
	//
	b3Capsule() { }

	// 
	b3Capsule(const b3Vec3& v1, const b3Vec3& v2, float32 r)
	{
		vertices[0] = v1;
		vertices[1] = v2;
		radius = r;
	}

	//
	~b3Capsule() { }

	b3Vec3 vertices[2];
	float32 radius;

	const b3Vec3& GetVertex(u32 index) const;
	u32 GetSupportVertex(const b3Vec3& direction) const;
};

// Unit capsule centered at the origin
extern const b3Capsule b3Capsule_identity;

inline const b3Vec3& b3Capsule::GetVertex(u32 index) const
{
	return vertices[index];
}

inline u32 b3Capsule::GetSupportVertex(const b3Vec3& d) const
{
	if (b3Dot(d, vertices[0]) > b3Dot(d, vertices[1]))
	{
		return 0;
	}
	return 1;
}

#endif