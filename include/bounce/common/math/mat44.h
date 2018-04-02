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

#ifndef B3_MAT_44_H
#define B3_MAT_44_H

#include <bounce/common/math/vec4.h>

// A 4-by-4 matrix stored in column-major order.
struct b3Mat44
{
	b3Mat44() { }

	b3Mat44(const b3Vec4& _x, const b3Vec4& _y, const b3Vec4& _z, const b3Vec4& _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	void SetZero()
	{
		x.SetZero();
		y.SetZero();
		z.SetZero();
		w.SetZero();
	}

	b3Vec4 x, y, z, w;
};

// 
inline b3Mat44 operator*(float32 s, const b3Mat44& A)
{
	return b3Mat44(s * A.x, s * A.y, s * A.z, s * A.w);
}

// 
inline b3Vec4 operator*(const b3Mat44& A, const b3Vec4& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z + v.w * A.w;
}

// 
inline b3Mat44 operator*(const b3Mat44& A, const b3Mat44& B)
{
	return b3Mat44(A * B.x, A * B.y, A * B.z, A * B.w);
}

#endif