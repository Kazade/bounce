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

#include <bounce/collision/sat/sat_hull_and_vertex.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/sphere.h>

scalar b3ProjectVertex(const b3Sphere* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Sphere* hull2)
{
	// Perform computations in the local space of the first hull.
	b3Vec3 support = b3MulT(xf1, b3Mul(xf2, hull2->vertex));

	u32 maxIndex = 0;
	scalar maxSeparation = -B3_MAX_SCALAR;

	for (u32 i = 0; i < hull1->faceCount; ++i)
	{
		b3Plane plane = hull1->GetPlane(i);
		scalar separation = b3Distance(support, plane);

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
