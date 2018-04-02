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

#ifndef B3_VEC_4_H
#define B3_VEC_4_H

#include <bounce/common/math/math.h>

// A 4D column vector.
struct b3Vec4
{
	//
	b3Vec4() { }

	//
	b3Vec4(float32 _x, float32 _y, float32 _z, float32 _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	//
	void SetZero()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		w = 0.0f;
	}
	
	float32 x, y, z, w;
};

//
inline b3Vec4 operator+(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

//
inline b3Vec4 operator-(const b3Vec4& a, const b3Vec4& b)
{
	return b3Vec4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

//
inline b3Vec4 operator*(float32 s, const b3Vec4& v)
{
	return b3Vec4(s * v.x, s * v.y, s * v.z, s * v.w);
}

//  
inline float32 operator*(const b3Vec4& A, const b3Vec4& B)
{
	return A.x * B.x + A.y * B.y + A.z * B.z + A.w * B.w;
}

#endif