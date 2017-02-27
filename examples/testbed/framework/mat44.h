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

#ifndef MAT44_H
#define MAT44_H

#include <bounce/bounce.h>

struct Vec4
{
	Vec4() { }
	Vec4(float32 _x, float32 _y, float32 _z, float32 _w) : x(_x), y(_y), z(_z), w(_w) { }

	void SetZero()
	{
		x = y = z = w = 0.0f;
	}

	void Set(float32 _x, float32 _y, float32 _z, float32 _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	float32 x, y, z, w;
};

inline Vec4 operator+(const Vec4& a, const Vec4& b)
{
	return Vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

inline Vec4 operator*(float32 s, const Vec4& v)
{
	return Vec4(s * v.x, s * v.y, s * v.z, s * v.w);
}

struct Mat44
{	
	Mat44() { }
	Mat44(const Vec4& _x, const Vec4& _y, const Vec4& _z, const Vec4& _w) : x(_x), y(_y), z(_z), w(_w) { }
	
	void SetIdentity()
	{
		x.Set(1.0f, 0.0f, 0.0f, 0.0f);
		y.Set(0.0f, 1.0f, 0.0f, 0.0f);
		z.Set(0.0f, 0.0f, 1.0f, 0.0f);
		w.Set(0.0f, 0.0f, 0.0f, 1.0f);
	}

	Vec4 x, y, z, w;
};

inline Vec4 operator*(const Mat44& A, const Vec4& v)
{
	return v.x * A.x + v.y * A.y + v.z * A.z + v.w * A.w;
}

inline b3Vec3 operator*(const Mat44& A, const b3Vec3& v)
{
	Vec4 q = v.x * A.x + v.y * A.y + v.z * A.z + A.w;
	return b3Vec3(q.x, q.y, q.z);
}

inline Mat44 operator*(const Mat44& A, const Mat44& B)
{
	return Mat44(A * B.x, A * B.y, A * B.z, A * B.w);
}

inline Mat44 GetMat44(const b3Transform& T)
{
	return Mat44(
		Vec4(T.rotation.x.x, T.rotation.x.y, T.rotation.x.z, 0.0f),
		Vec4(T.rotation.y.x, T.rotation.y.y, T.rotation.y.z, 0.0f),
		Vec4(T.rotation.z.x, T.rotation.z.y, T.rotation.z.z, 0.0f),
		Vec4(T.position.x, T.position.y, T.position.z, 1.0f));
}

inline b3Transform GetTransform(const Mat44& T)
{
	b3Transform xf;
	xf.rotation.x.Set(T.x.x, T.x.y, T.x.z);
	xf.rotation.y.Set(T.y.x, T.y.y, T.y.z);
	xf.rotation.z.Set(T.z.x, T.z.y, T.z.z);
	xf.position.Set(T.w.x, T.w.y, T.w.z);
	return xf;
}

inline float32 RandomFloat(float32 a, float32 b)
{
	float32 x = float32(rand()) / float32(RAND_MAX);
	float32 diff = b - a;
	float32 r = x * diff;
	return a + r;
}

struct Ray3
{
	b3Vec3 Start() const
	{
		return origin;
	}

	b3Vec3 End() const
	{
		return origin + fraction * direction;
	}

	b3Vec3 direction;
	b3Vec3 origin;
	float32 fraction;
};

#endif