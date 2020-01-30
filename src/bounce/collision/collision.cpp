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

#include <bounce/collision/collision.h>

#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/box_hull.h>
#include <bounce/collision/shapes/cylinder_hull.h>
#include <bounce/collision/shapes/cone_hull.h>

const b3Sphere b3Sphere_identity(b3Vec3_zero, scalar(1)); 

const b3Capsule b3Capsule_identity(b3Vec3(scalar(0), scalar(-0.5), scalar(0)), b3Vec3(scalar(0), scalar(0.5), scalar(0)), scalar(1));

const b3BoxHull b3BoxHull_identity(scalar(1), scalar(1), scalar(1));

const b3CylinderHull b3CylinderHull_identity(scalar(1), scalar(1));

const b3ConeHull b3ConeHull_identity(scalar(1), scalar(1));

bool b3RayCast(b3RayCastOutput* output,
	const b3RayCastInput* input,
	const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3)
{
	b3Vec3 p1 = input->p1;
	b3Vec3 p2 = input->p2;
	scalar maxFraction = input->maxFraction;
	
	b3Vec3 d = p2 - p1;
	
	if (b3LengthSquared(d) < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	scalar len = b3Length(n);
	
	if (len == scalar(0))
	{
		return false;
	}

	n /= len;

	scalar num = b3Dot(n, v1 - p1);
	scalar den = b3Dot(n, d);

	if (den == scalar(0))
	{
		return false;
	}

	scalar fraction = num / den;

	// Is the intersection not on the segment?
	if (fraction < scalar(0) || maxFraction < fraction)
	{
		return false;
	}

	b3Vec3 Q = p1 + fraction * d;

	b3Vec3 A = v1;
	b3Vec3 B = v2;
	b3Vec3 C = v3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	// Barycentric coordinates for Q
	scalar u = b3Dot(QB_x_QC, AB_x_AC);
	scalar v = b3Dot(QC_x_QA, AB_x_AC);
	scalar w = b3Dot(QA_x_QB, AB_x_AC);

	// Is the intersection on the triangle?
	if (u >= scalar(0) && v >= scalar(0) && w >= scalar(0))
	{
		output->fraction = fraction;

		// Does the ray start from below or above the triangle?
		if (num > scalar(0))
		{
			output->normal = -n;
		}
		else
		{
			output->normal = n;
		}

		return true;
	}

	return false;
}
