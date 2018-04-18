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

#include <bounce/common/math/math.h>
#include <bounce/common/math/mat33.h>

// A quaternion can represent an orientation with 4 scalars.
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
		Set(axis, angle);
	}
	
	// Write an indexed value to this quaternion.
	float32& operator[](u32 i)
	{
		return (&x)[i];
	}

	// Read an indexed value from this quaternion.
	float32 operator[](u32 i) const
	{
		return (&x)[i];
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
	
	// Convert this quaternion to the unit quaternion. Return the length.
	float32 Normalize()
	{
		float32 length = b3Sqrt(x * x + y * y + z * z + w * w);
		if (length > B3_EPSILON)
		{
			float32 s = 1.0f / length;
			x *= s;
			y *= s;
			z *= s;
			w *= s;
		}
		return length;
	}

	// Set this quaternion from an axis and full angle 
	// of rotation about the axis.
	void Set(const b3Vec3& axis, float32 angle)
	{
		// half angle
		float32 theta = 0.5f * angle;
		
		float32 sine = sin(theta);
		x = sine * axis.x;
		y = sine * axis.y;
		z = sine * axis.z;

		w = cos(theta);
	}

	// If this quaternion represents an orientation output 
	// the axis and angle of rotation about the axis.
	void GetAxisAngle(b3Vec3* axis, float32* angle) const
	{
		// sin^2 = 1 - cos^2
		// sin = sqrt( sin^2 ) = ||v||
		// axis = v / sin
		b3Vec3 v(x, y, z);
		float32 sine = b3Length(v);
		axis->SetZero();
		if (sine > B3_EPSILON)
		{
			float32 s = 1.0f / sine;
			*axis = s * v;
		}
		
		// cosine check
		float32 cosine = b3Clamp(w, -1.0f, 1.0f);
		// half angle
		float32 theta = acos(cosine);
		// full angle
		*angle = 2.0f * theta;
	}

	float32 x, y, z, w;
};

// Identity quaternion
extern const b3Quat b3Quat_identity;

// Add two quaternions.
inline b3Quat operator+(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

// Subtract two quaternions.
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

// Multiply two quaternions.
inline b3Quat b3Mul(const b3Quat& a, const b3Quat& b)
{
	return b3Quat(
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
		a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

// Multiply two quaternions.
inline b3Quat operator*(const b3Quat& a, const b3Quat& b)
{
	return b3Mul(a, b);
}

// Perform the dot poduct of two quaternions.
inline float32 b3Dot(const b3Quat& a, const b3Quat& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// Return the conjugate of a quaternion.
// If the quaternion is unit this returns its inverse.
inline b3Quat b3Conjugate(const b3Quat& q)
{
	return b3Quat(-q.x, -q.y, -q.z, q.w);
}

// Multiply the conjugate of a quaternion times another quaternion.
inline b3Quat b3MulT(const b3Quat& a, const b3Quat& b)
{
	return b3Mul(b3Conjugate(a), b);
}

// Return the length of a quaternion.
inline float32 b3Length(const b3Quat& q)
{
	return b3Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

// Convert a quaternion to the unit quaternion.
inline b3Quat b3Normalize(const b3Quat& q)
{
	float32 s = b3Length(q);
	if (s > B3_EPSILON)
	{
		s = 1.0f / s;
		return s * q;
	}
	return b3Quat(0.0f, 0.0f, 0.0f, 1.0f);
}

// Rotate a vector.
inline b3Vec3 b3Mul(const b3Quat& q, const b3Vec3& v)
{
	b3Vec3 qv(q.x, q.y, q.z);
	float32 qs = q.w;
	
	b3Vec3 t = 2.0f * b3Cross(qv, v);
	return v + qs * t + b3Cross(qv, t);
}

// Inverse rotate a vector.
inline b3Vec3 b3MulT(const b3Quat& q, const b3Vec3& v)
{
	return b3Mul(b3Conjugate(q), v);
}

// Convert a 3-by3 rotation matrix to a rotation quaternion.
inline b3Quat b3Mat33Quat(const b3Mat33& m)
{
	// Check the diagonal.
	float32 trace = m[0][0] + m[1][1] + m[2][2];
	
	if (trace > 0.0f) 
	{
		b3Quat result;
		
		float32 s = b3Sqrt(trace + 1.0f);
		result.w = 0.5f * s;
				
		float32 t = 0.5f / s;
		result.x = t * (m[1][2] - m[2][1]);
		result.y = t * (m[2][0] - m[0][2]);
		result.z = t * (m[0][1] - m[1][0]);
		return result;
	}

	// Diagonal is negative.
	const u32 next[3] = { 1, 2, 0 };
	
	u32 i = 0;
	
	if (m[1][1] > m[0][0])
	{
		i = 1;
	}

	if (m[2][2] > m[i][i])
	{
		i = 2;
	}

	u32 j = next[i];
	u32 k = next[j];

	float32 s = sqrt((m[i][i] - (m[j][j] + m[k][k])) + 1.0f);
	
	float32 q[4];
	q[i] = s * 0.5f;
	
	float32 t;
	if (s != 0.0f) 
	{
		t = 0.5f / s;
	}
	else
	{
		t = s;
	}

	q[3] = t * (m[j][k] - m[k][j]);
	q[j] = t * (m[i][j] + m[j][i]);
	q[k] = t * (m[i][k] + m[k][i]);
		
	b3Quat result;
	result.x = q[0];
	result.y = q[1];
	result.z = q[2];
	result.w = q[3];
	return result;
}

// Convert a rotation quaternion to a 3-by-3 rotation matrix.
inline b3Mat33 b3QuatMat33(const b3Quat& q)
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

// Rotation about the x-axis.
inline b3Quat b3QuatRotationX(float32 angle)
{
	float32 x = 0.5f * angle;

	b3Quat q;
	q.x = sin(x);
	q.y = 0.0f;
	q.z = 0.0f;
	q.w = cos(x);
	return q;
}

// Rotation about the y-axis.
inline b3Quat b3QuatRotationY(float32 angle)
{
	float32 x = 0.5f * angle;

	b3Quat q;
	q.x = 0.0f;
	q.y = sin(x);
	q.z = 0.0f;
	q.w = cos(x);
	return q;
}

// Rotation about the z-axis.
inline b3Quat b3QuatRotationZ(float32 angle)
{
	float32 x = 0.5f * angle;

	b3Quat q;
	q.x = 0.0f;
	q.y = 0.0f;
	q.z = sin(x);
	q.w = cos(x);
	return q;
}

#endif