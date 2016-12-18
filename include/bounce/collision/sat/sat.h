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

#ifndef B3_SAT_H
#define B3_SAT_H

#include <bounce\common\geometry.h>

struct b3Hull;

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3FaceQuery
{
	u32 index;
	float32 separation;
};

float32 b3Project(const b3Hull* hull, const b3Plane& plane);

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB);

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3EdgeQuery
{
	u32 indexA;
	u32 indexB;
	float32 separation;
};

bool b3IsMinkowskiFace(const b3Vec3& A, const b3Vec3& B, const b3Vec3& B_x_A, const b3Vec3& C, const b3Vec3& D, const b3Vec3& D_x_C);

float32 b3Project(const b3Vec3& P1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C1);

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xfA, const b3Hull* hullA,
	const b3Transform& xfB, const b3Hull* hullB);

#endif
