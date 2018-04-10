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

#ifndef B3_VEC2_H
#define B3_VEC2_H

#include <bounce/common/math/math.h>

// A 2D column vector.
struct b3Vec2
{
	// Does nothing for performance.
	b3Vec2() { }

	// Set this vector from two components.
	b3Vec2(float32 _x, float32 _y) : x(_x), y(_y) { }

	// Read an indexed component from this vector.
	float32 operator[](u32 i) const { return (&x)[i]; }

	// Write an indexed component to this vector.
	float32& operator[](u32 i) { return (&x)[i]; }

	// Add a vector to this vector.
	void operator+=(const b3Vec2& v)
	{
		x += v.x;
		y += v.y;
	}

	// Subtract a vector from this vector.
	void operator-=(const b3Vec2& v)
	{
		x -= v.x;
		y -= v.y;
	}

	// Scale this vector.
	void operator*=(float32 s)
	{
		x *= s;
		y *= s;
	}

	// Scale this vector.
	void operator/=(float32 s)
	{
		x /= s;
		y /= s;
	}

	// Set this vector to the zero vector.
	void SetZero()
	{
		x = y = 0.0f;
	}

	// Set this vector from two components.
	void Set(float32 _x, float32 _y)
	{
		x = _x;
		y = _y;
	}

	// Normalize this vector.
	void Normalize()
	{
		float32 s = b3Sqrt(x * x + y * y);
		if (s > B3_EPSILON)
		{
			x /= s;
			y /= s;
		}
	}

	float32 x, y;
};

// Zero vector 
extern const b3Vec2 b3Vec2_zero;

// Left vector 
extern const b3Vec2 b3Vec2_x;

// Right vector 
extern const b3Vec2 b3Vec2_y;

// Negate a vector.
inline b3Vec2 operator-(const b3Vec2& v)
{
	return b3Vec2(-v.x, -v.y);
}

// Add two vectors.
inline b3Vec2 operator+(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x + b.x, a.y + b.y);
}

// Subtract two vectors.
inline b3Vec2 operator-(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x - b.x, a.y - b.y);
}

// Multiply a vector by a scalar.
inline b3Vec2 operator*(float32 s, const b3Vec2& v)
{
	return b3Vec2(s * v.x, s * v.y);
}

// Multiply a vector by a scalar.
inline b3Vec2 operator*(const b3Vec2& v, float32 s)
{
	return s * v;
}

// Compute the dot product of two vectors.
inline float32 b3Dot(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

// Compute the length of a vector.
inline float32 b3Length(const b3Vec2& v)
{
	return b3Sqrt(v.x * v.x + v.y * v.y);
}

// Normalize a vector.
inline b3Vec2 b3Normalize(const b3Vec2& v)
{
	float32 length = b3Length(v);
	if (length > B3_EPSILON)
	{
		float32 s = 1.0f / length;
		return s * v;
	}
	return v;
}

// Compute the euclidean distance between two points.
inline float32 b3Distance(const b3Vec2& a, const b3Vec2& b)
{
	return b3Length(a - b);
}

// Compute the determinant of two 2D vectors.
inline float32 b3Det(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

#endif
