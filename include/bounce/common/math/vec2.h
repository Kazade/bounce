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

#ifndef B3_VEC2_H
#define B3_VEC2_H

#include <bounce/common/math/math.h>

// A 2D column vector.
struct b3Vec2
{
	// Does nothing for performance.
	b3Vec2() { }

	// Set this vector from two components.
	b3Vec2(scalar _x, scalar _y) : x(_x), y(_y) { }

	// Read an indexed component from this vector.
	scalar operator[](u32 i) const { return (&x)[i]; }

	// Write an indexed component to this vector.
	scalar& operator[](u32 i) { return (&x)[i]; }

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
	void operator*=(scalar s)
	{
		x *= s;
		y *= s;
	}

	// Scale this vector.
	void operator/=(scalar s)
	{
		x /= s;
		y /= s;
	}

	// Set this vector to the zero vector.
	void SetZero()
	{
		x = y = scalar(0);
	}

	// Set this vector from two components.
	void Set(scalar _x, scalar _y)
	{
		x = _x;
		y = _y;
	}

	// Normalize this vector.
	void Normalize()
	{
		scalar s = b3Sqrt(x * x + y * y);
		if (s > B3_EPSILON)
		{
			x /= s;
			y /= s;
		}
	}

	scalar x, y;
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
inline b3Vec2 operator*(scalar s, const b3Vec2& v)
{
	return b3Vec2(s * v.x, s * v.y);
}

// Multiply a vector by a scalar.
inline b3Vec2 operator*(const b3Vec2& v, scalar s)
{
	return s * v;
}

// Compute the dot product of two vectors.
inline scalar b3Dot(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

// Compute the length of a vector.
inline scalar b3Length(const b3Vec2& v)
{
	return b3Sqrt(v.x * v.x + v.y * v.y);
}

// Compute the squared length of a vector.
inline scalar b3LengthSquared(const b3Vec2& v)
{
	return v.x * v.x + v.y * v.y;
}

// Normalize a vector.
inline b3Vec2 b3Normalize(const b3Vec2& v)
{
	scalar length = b3Length(v);
	if (length > B3_EPSILON)
	{
		scalar s = scalar(1) / length;
		return s * v;
	}
	return v;
}

// Compute the euclidean distance between two points.
inline scalar b3Distance(const b3Vec2& a, const b3Vec2& b)
{
	return b3Length(a - b);
}

// Compute the determinant of two 2D vectors.
inline scalar b3Det(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

#endif
