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

#ifndef B3_VEC_3_H
#define B3_VEC_3_H

#include <bounce/common/math/math.h>

// A 3D column vector.
struct b3Vec3 
{
	// Does nothing for performance.
	b3Vec3() { }

	// Set this vector from three components.
	b3Vec3(float32 _x, float32 _y, float32 _z) : x(_x), y(_y), z(_z) { }

	// Read an indexed component from this vector.
	float32 operator[](u32 i) const
	{
		return (&x)[i];
	}

	// Write an indexed component to this vector.
	float32& operator[](u32 i)
	{
		return (&x)[i];
	}

	// Add a vector to this vector.
	void operator+=(const b3Vec3& b) 
	{
		x += b.x;
		y += b.y;
		z += b.z;
	}

	// Subtract this vector from another vector.
	void operator-=(const b3Vec3& b) 
	{
		x -= b.x;
		y -= b.y;
		z -= b.z;
	}

	// Scale this vector.
	void operator*=(float32 s) 
	{
		x *= s;
		y *= s;
		z *= s;
	}
	
	// Scale this vector.
	void operator/=(float32 a)
	{
		float32 s = 1.0f / a;
		x *= s;
		y *= s;
		z *= s;
	}

	// Set this vector to the zero vector.
	void SetZero() 
	{
		x = y = z = 0.0f;
	}

	// Set this vector from three coordinates.
	void Set(float32 _x, float32 _y, float32 _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	// Convert this vector to the unit vector. Return the length.
	float32 Normalize()
	{
		float32 length = b3Sqrt(x * x + y * y + z * z);
		if (length > B3_EPSILON)
		{
			float32 s = 1.0f / length;
			x *= s;
			y *= s;
			z *= s;
		}
		return length;
	}

	float32 x, y, z;
};

// Zero vector
extern const b3Vec3 b3Vec3_zero;

// Right vector
extern const b3Vec3 b3Vec3_x;

// Up vector 
extern const b3Vec3 b3Vec3_y;

// Forward vector 
extern const b3Vec3 b3Vec3_z;

// Negate a vector.
inline b3Vec3 operator-(const b3Vec3& v) 
{
	return b3Vec3(-v.x, -v.y, -v.z);
}

// Compute the sum of two vectors.
inline b3Vec3 operator+(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

// Compute the subtraction of two vectors.
inline b3Vec3 operator-(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Compute a scalar-vector product.
inline b3Vec3 operator*(float32 s, const b3Vec3& v) 
{
	return b3Vec3(s * v.x, s * v.y, s * v.z);
}

// Compute a scalar-vector product.
inline b3Vec3 operator*(const b3Vec3& v, float32 s)
{
	return s * v;
}

// Inverse multiply a scalar-vector.
inline b3Vec3 operator/(const b3Vec3& v, float32 s)
{
	return b3Vec3(v.x / s, v.y / s, v.z / s);
}

// Compute the dot-product of two vectors.
inline float32 b3Dot(const b3Vec3& a, const b3Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Compute the cross-product of two vectors.
inline b3Vec3 b3Cross(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

// Compute the determinant of a matrix whose columns are three given vectors.
// Useful property: det(a, b, c) = det(c, a, b) = det(b, c, a).
inline float32 b3Det(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c)
{
	return b3Dot(a, b3Cross(b, c));
}

// Compute the length of a vector.
inline float32 b3Length(const b3Vec3& v) 
{
	return b3Sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Compute the squared length of a vector.
inline float32 b3LengthSquared(const b3Vec3& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

// Compute the normalized vector of a (non-zero!) vector.
inline b3Vec3 b3Normalize(const b3Vec3& v)
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
inline float32 b3Distance(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Length(a - b);
}

// Compute the squared distance between two points.
inline float32 b3DistanceSquared(const b3Vec3& a, const b3Vec3& b)
{
	b3Vec3 v = a - b;
	return b3LengthSquared(v);
}

// Compute the triangle area.
inline float32 b3Area(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c)
{
	return 0.5f * b3Length(b3Cross(b - a, c - a));
}

// Compute the squared triangle area.
inline float32 b3AreaSquared(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c)
{
	return 0.25f * b3LengthSquared(b3Cross(b - a, c - a));
}

// Compute the tetrahedron volume.
inline float32 b3Volume(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c, const b3Vec3& d)
{
	float32 volume = b3Det(b - a, c - a, d - a);
	// Force a positive volume.
	float32 sign = b3Sign(volume);
	const float32 inv6 = 1.0f / 6.0f;
	return sign * inv6 * volume;
}

// Compute the squared tetrahedron volume.
inline float32 b3VolumeSquared(const b3Vec3& a, const b3Vec3& b, const b3Vec3& c, const b3Vec3& d)
{
	float32 volume = b3Volume(a, b, c, d);
	return volume * volume;
}

// Compute the minimum vector between two vector (per-element).
inline b3Vec3 b3Min(const b3Vec3& a, const b3Vec3& b) 
{
	return b3Vec3(b3Min(a.x, b.x), b3Min(a.y, b.y), b3Min(a.z, b.z));
}

// Compute the maximum vector between two vector (per-element).
inline b3Vec3 b3Max(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(b3Max(a.x, b.x), b3Max(a.y, b.y), b3Max(a.z, b.z));
}

// Find a perpendicular vector to a vector.
inline b3Vec3 b3Perp(const b3Vec3& v)
{
	// Box2D
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component of a
	// unit vector must be greater or equal to 0.57735.
	if (b3Abs(v.x) >= float32(0.57735027))
	{
		return b3Vec3(v.y, -v.x, 0.0f);
	}

	return b3Vec3(0.0f, v.z, -v.y);
}

#endif