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

#ifndef B3_Q_HULL_H
#define B3_Q_HULL_H

#include <bounce/collision/shapes/hull.h>

struct b3QHull : public b3Hull
{
	b3QHull();
	~b3QHull();

	// Create a convex hull from a point list.
	// If the point list defines a degenerate polyhedron 
	// the old hull is not cleared.
	//
	// Coincident points are removed.
	// Coplanar faces are merged.
	void Set(const b3Vec3* points, u32 count);

	// Set this hull as a cylinder located at the origin.
	void SetAsCylinder(float32 radius = 1.0f, float32 height = 1.0f);

	// Set this hull as a cone located at the origin.
	void SetAsCone(float32 radius = 1.0f, float32 height = 1.0f);
};

#endif