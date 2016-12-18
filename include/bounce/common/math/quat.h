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

#ifndef B3_QUAT_H
#define B3_QUAT_H

#include <bounce\common\math\math.h>
#include <bounce\common\math\mat33.h>

// A quaternion represents an orientation with 4 real numbers.
struct b3Quat 
{
	// Default constructor does nothing for performance.
	b3Quat() { }

	// Set this quaternion from four values.
	b3Quat(float32 _x, float32 _y, float32 _z, float32 _w) : x(_x), y(_y), z(_z), w(_w) { }

	// Set this quaternion from an axis and an angle 
	// of rotation about the axis.
	b3Quat(const b3Vec3& axis, float32 angle)
	{
		float32 theta = 0.5f * angle;
		float32 s = sin(theta);
		x = s * axis.x;
		y = s * axis.y;
		z = s * axis.z;
		w = cos(theta);
	}

	// Add a quaternion to this quaternion.
	void operator+=(const b3Quat& q)
	{
		x += q.x;
		y += q.y;
		z += q.z;
		w += q.w;
	}

	// Subtract a quaternion from this quaternion.
	void operator-=(const b3Quat& q)
	{
		x -= q.x;
		y -= q.y;
		z -= q.z;
		w -= q.w;
	}

	// Set this quaternion to identity.
	void SetIdentity() 
	{
		x = y = z = 0.0f; 
		w = 1.0f;
	}

	// Set this quaternion from four values.
	void Set(float32 _x, float32 _y, float32 _z, float32 _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}
	
	// Set this quaternion from an axis and an angle 
	// of rotation about the axis.
	void Set(const b3Vec3& axis, float32 angle)
	{
		float32 theta = 0.5f * angle;
		float32 s = sin(theta);
		x = s * axis.x;
		y = s * axis.y;
		z = s * axis.z;
		w = cos(theta);
	}

	// Normalize this quaternion.
	void Normalize()
	{
		float32 s = b3Sqrt(x * x + y * y + z * z + w * w);
		if (s != 0.0f)
		{
			x /= s;
			y /= s;
			z /= s;
			w /= s;
		}
	}

	float32 x, y, z, w;
};

// Add two quaternions.
inline b3Quat operator+(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

// Sobtract two quaternions.
inline b3Quat operator-(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

// Multiply a quaternion by a scalar.
inline b3Quat operator*(float32 s, const b3Quat& q)
{
	return b3Quat(s * q.x, s * q.y, s * q.z, s * q.w);
}

// Negate a quaternion.
inline b3Quat operator-(const b3Quat& q)
{
	return b3Quat(-q.x, -q.y, -q.z, -q.w);
}

// Compute a quaternion-quaternion product.
inline b3Quat operator*(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
		a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

// Compute the length of a quaternion.
inline float32 b3Length(const b3Quat& q)
{
	return b3Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

// Normalize a quarternion.
inline b3Quat b3Normalize(const b3Quat& q)
{
	float32 s = b3Length(q);
	if (s != 0.0f)
	{
		s = 1.0f / s;
		return s * q;
	}
	return b3Quat(0.0f, 0.0f, 0.0f, 1.0f);
}

// Compute the dot poduct of two quaternions.
inline float b3Dot(const b3Quat& a, const b3Quat& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Compute the conjugate of a quaternion.
inline b3Quat b3Conjugate(const b3Quat& q)
{
	return b3Quat(-q.x, -q.y, -q.z, q.w);
}

// Rotate a vector by an orientation quaternion.
inline b3Vec3 b3Mul(const b3Quat& q, const b3Vec3& v)
{
	b3Vec3 qv(q.x, q.y, q.z);
	float32 qs = q.w;
	
	b3Vec3 t = 2.0f * b3Cross(qv, v);
	return v + qs * t + b3Cross(qv, t);
}

// Convert a quaternion to a 3-by-3 rotation matrix.
inline b3Mat33 b3ConvertQuatToRot(const b3Quat& q)
{
	float32 x = q.x, y = q.y, z = q.z, w = q.w;
	float32 x2 = x + x, y2 = y + y, z2 = z + z;
	float32 xx = x * x2, xy = x * y2, xz = x * z2;
	float32 yy = y * y2, yz = y * z2, zz = z * z2;
	float32 wx = w * x2, wy = w * y2, wz = w * z2;

	return b3Mat33(
		b3Vec3(1.0f - (yy + zz),          xy + wz, xz - wy),
		b3Vec3(         xy - wz, 1.0f - (xx + zz), yz + wx),
		b3Vec3(         xz + wy,          yz - wx, 1.0f - (xx + yy)));
}

// Perform a linear interpolation between two quaternions.
inline b3Quat b3Lerp(const b3Quat& a, const b3Quat& b, float32 fraction)
{
	B3_ASSERT(fraction >= 0.0f);
	B3_ASSERT(fraction <= 1.0f);
	float32 w1 = 1.0f - fraction;
	float32 w2 = fraction;
	return w1 * a + w2 * b;
}

// Perform a spherical interpolation between two quaternions.
inline b3Quat b3Slerp(const b3Quat& a, const b3Quat& b, float32 fraction)
{
	B3_ASSERT(fraction >= 0.0f);
	B3_ASSERT(fraction <= 1.0f);
	float32 w1 = 1.0f - fraction;
	float32 w2 = fraction;

	float32 cosine = b3Dot(a, b);
	b3Quat b2 = b;
	if (cosine <= FLT_EPSILON * FLT_EPSILON)
	{
		b2 = -b;
		cosine = -cosine;
	}

	if (cosine > 1.0f - FLT_EPSILON)
	{
		return w1 * a + w2 * b2;
	}

	float32 angle = acos(cosine);
	float32 sine = sin(angle);
	b3Quat q1 = sin(w1 * angle) * a;
	b3Quat q2 = sin(w2 * angle) * b2;
	float32 invSin = 1.0f / sine;

	return invSin * (q1 + q2);
}

#endif
