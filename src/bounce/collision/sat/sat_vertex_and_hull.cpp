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

#include <bounce/collision/sat/sat_vertex_and_hull.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/hull.h>

float32 b3ProjectVertex(const b3Sphere* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Sphere* hull,
	const b3Transform& xf2, const b3Hull* hull2)
{
	// Perform computations in the local space of the second hull.
	b3Vec3 support = b3MulT(xf2, b3Mul(xf1, hull->vertex));

	u32 maxIndex = 0;
	float32 maxSeparation = -B3_MAX_FLOAT;

	for (u32 i = 0; i < hull2->faceCount; ++i)
	{
		b3Plane plane = hull2->GetPlane(i);
		float32 separation = b3Distance(support, plane);

		if (separation > maxSeparation)
		{
			maxIndex = i;
			maxSeparation = separation;
		}
	}
	
	b3FaceQuery out;
	out.index = maxIndex;
	out.separation = maxSeparation;
	return out;
}
