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

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/mat44.h>
#include <bounce/common/math/transform.h>

const b3Vec2 b3Vec2_zero(0.0f, 0.0f);
const b3Vec2 b3Vec2_x(1.0f, 0.0f);
const b3Vec2 b3Vec2_y(0.0f, 1.0f);

const b3Vec3 b3Vec3_zero(0.0f, 0.0f, 0.0f);
const b3Vec3 b3Vec3_x(1.0f, 0.0f, 0.0f);
const b3Vec3 b3Vec3_y(0.0f, 1.0f, 0.0f);
const b3Vec3 b3Vec3_z(0.0f, 0.0f, 1.0f);

const b3Mat22 b3Mat22_zero(
	b3Vec2(0.0f, 0.0f),
	b3Vec2(0.0f, 0.0f));

const b3Mat22 b3Mat22_identity(
	b3Vec2(1.0f, 0.0f),
	b3Vec2(0.0f, 1.0f));

const b3Mat33 b3Mat33_zero(
	b3Vec3(0.0f, 0.0f, 0.0f),
	b3Vec3(0.0f, 0.0f, 0.0f),
	b3Vec3(0.0f, 0.0f, 0.0f));

const b3Mat33 b3Mat33_identity(
	b3Vec3(1.0f, 0.0f, 0.0f),
	b3Vec3(0.0f, 1.0f, 0.0f),
	b3Vec3(0.0f, 0.0f, 1.0f));

const b3Transform b3Transform_identity(b3Mat33_identity, b3Vec3_zero);

const b3Quat b3Quat_identity(0.0f, 0.0f, 0.0f, 1.0f);

b3Vec2 b3Mat22::Solve(const b3Vec2& b) const
{
	// Cramer's rule
	float32 a11 = x.x, a12 = y.x;
	float32	a21 = x.y, a22 = y.y;

	float32 det = a11 * a22 - a12 * a21;

	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	b3Vec2 xn;
	xn.x = det * (a22 * b.x - a12 * b.y);
	xn.y = det * (a11 * b.y - a21 * b.x);
	return xn;
}

b3Mat22 b3Inverse(const b3Mat22& A)
{
	float32 a11 = A.x.x, a12 = A.y.x;
	float32	a21 = A.x.y, a22 = A.y.y;

	float32 det = a11 * a22 - a12 * a21;
	
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	
	b3Mat22 B;
	B.x.x = det * a22;	B.y.x = -det * a12;
	B.x.y = -det * a21;	B.y.y = det * a11;
	return B;
}

b3Vec3 b3Mat33::Solve(const b3Vec3& b) const
{
	// Cramer's rule
	float32 det = b3Det(x, y, z);
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	
	b3Vec3 xn;
	xn.x = det * b3Det(b, y, z);
	xn.y = det * b3Det(x, b, z);
	xn.z = det * b3Det(x, y, b);
	return xn;
}

static B3_FORCE_INLINE b3Mat33 b3Adjucate(const b3Mat33& A)
{
	b3Vec3 c1 = b3Cross(A.y, A.z);
	b3Vec3 c2 = b3Cross(A.z, A.x);
	b3Vec3 c3 = b3Cross(A.x, A.y);

	b3Mat33 B;
	B.x.x = c1.x; B.x.y = c2.x; B.x.z = c3.x;
	B.y.x = c1.y; B.y.y = c2.y; B.y.z = c3.y;
	B.z.x = c1.z; B.z.y = c2.z; B.z.z = c3.z;
	return B;
}

b3Mat33 b3Inverse(const b3Mat33& A)
{
	// Cofactor method
	float32 det = b3Det(A.x, A.y, A.z);
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	return det * b3Adjucate(A);
}

b3Mat33 b3SymInverse(const b3Mat33& A)
{
	float32 det = b3Det(A.x, A.y, A.z);
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	float32 a11 = A.x.x, a12 = A.y.x, a13 = A.z.x;
	float32 a22 = A.y.y, a23 = A.z.y;
	float32 a33 = A.z.z;

	b3Mat33 M;

	M.x.x = det * (a22 * a33 - a23 * a23);
	M.x.y = det * (a13 * a23 - a12 * a33);
	M.x.z = det * (a12 * a23 - a13 * a22);

	M.y.x = M.x.y;
	M.y.y = det * (a11 * a33 - a13 * a13);
	M.y.z = det * (a13 * a12 - a11 * a23);

	M.z.x = M.x.z;
	M.z.y = M.y.z;
	M.z.z = det * (a11 * a22 - a12 * a12);

	return M;
}