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

#ifndef B3_AABB_3_H
#define B3_AABB_3_H

#include <bounce/common/math/transform.h>

// A min-max representation of a three-dimensional AABB.
struct b3AABB3 
{
	b3Vec3 m_lower; // lower vertex
	b3Vec3 m_upper; // upper vertex

	// Get the support vertex in a given direction.
	b3Vec3 GetSupportVertex(const b3Vec3& direction) const
	{
		b3Vec3 support;
		support.x = direction.x < 0.0f ? m_lower.x : m_upper.x;
		support.y = direction.y < 0.0f ? m_lower.y : m_upper.y;
		support.z = direction.z < 0.0f ? m_lower.z : m_upper.z;
		return support;
	}

	// Compute this AABB from a list of points.
	void Compute(const b3Vec3* points, u32 count)
	{
		m_lower = m_upper = points[0];
		for (u32 i = 1; i < count; ++i)
		{
			m_lower = b3Min(m_lower, points[i]);
			m_upper = b3Max(m_upper, points[i]);
		}
	}

	// Compute this AABB from a list of points and a transform.
	void Compute(const b3Vec3* points, u32 count, const b3Transform& xf)
	{
		m_lower = m_upper = b3Mul(xf, points[0]);
		for (u32 i = 1; i < count; ++i)
		{
			b3Vec3 v = b3Mul(xf, points[i]);
			m_lower = b3Min(m_lower, v);
			m_upper = b3Max(m_upper, v);
		}
	}

	// Extend this AABB by a scalar.
	void Extend(float32 s) 
	{
		b3Vec3 r(s, s, s);
		m_lower -= r;
		m_upper += r;
	}
	
	// Extend this AABB by a radius vector.
	void Extend(const b3Vec3& r)
	{
		m_lower -= r;
		m_upper += r;
	}

	// Compute the centroid of this AABB.
	b3Vec3 Centroid() const 
	{
		return  0.5f * (m_lower + m_upper);
	}

	// Compute the width of this AABB.
	float32 Width() const 
	{
		return m_upper.x - m_lower.x;
	}

	// Compute the height of this AABB.
	float32 Height() const 
	{
		return m_upper.y - m_lower.y;
	}

	// Compute the depth of this AABB.
	float32 Depth() const 
	{
		return m_upper.z - m_lower.z;
	}

	// Compute the total of cubic units contained in this AABB.
	float32 Volume() const 
	{
		return Width() * Height() * Depth();
	}

	// Compute the surface area of this AABB.
	float32 SurfaceArea() const 
	{
		return 2.0f * (Width() * Depth() + Width() * Height() + Depth() * Height());
	}

	// Read the index of the longest axis of this AABB.
	u32 GetLongestAxisIndex() const
	{
		b3Vec3 c = Centroid();
		b3Vec3 r = m_upper - c;
		float32 max = r[0];
		u32 i = 0;
		if (r[1] > max)
		{
			max = r[1];
			i = 1;
		}
		if (r[2] > max)
		{
			max = r[2];
			i = 2;
		}
		return i;
	}

	// Test if this AABB contains a point.
	bool Contains(const b3Vec3& point) const
	{
		return	m_lower.x <= point.x && point.x <= m_upper.x &&
				m_lower.y <= point.y && point.y <= m_upper.y &&
				m_lower.z <= point.z && point.z <= m_upper.z;
	}

	// Test if this AABB contains another AABB.
	bool Contains(const b3AABB3& aabb) const
	{
		return Contains(aabb.m_lower) && Contains(aabb.m_upper);
	}

	// Test if a ray intersects this AABB.
	bool TestRay(float32& minFraction, const b3Vec3& p1, const b3Vec3& p2, float32 maxFraction) const
	{
		b3Vec3 d = p2 - p1;
		
		float32 lower = 0.0f;
		float32 upper = maxFraction;
		
		for (u32 i = 0; i < 3; ++i)
		{
			float32 numerators[2], denominators[2];

			numerators[0] = p1[i] - m_lower[i];
			numerators[1] = m_upper[i] - p1[i];

			denominators[0] = -d[i];
			denominators[1] = d[i];
			
			for (u32 j = 0; j < 2; ++j)
			{
				float32 numerator = numerators[j];
				float32 denominator = denominators[j];
				
				if (denominator == 0.0f)
				{
					// s is parallel to this half-space.
					if (numerator < 0.0f)
					{
						// s is outside of this half-space.
						// dot(n, p1) and dot(n, p2) < 0.
						return false;
					}
				}
				else
				{
					if (denominator < 0.0f)
					{
						// s enters this half-space.
						if (numerator < lower * denominator)
						{
							// Increase lower.
							lower = numerator / denominator;
						}
					}
					else
					{
						// s exits the half-space.	
						if (numerator < upper * denominator)
						{
							// Decrease upper.
							upper = numerator / denominator;
						}
					}
					// Exit if intersection becomes empty.
					if (upper < lower)
					{
						return false;
					}
				}
			}
		}

		B3_ASSERT(lower >= 0.0f && lower <= maxFraction);
		minFraction = lower;
		return true;
	}
};

// Compute an AABB that encloses two AABBs.
inline b3AABB3 b3Combine(const b3AABB3& a, const b3AABB3& b) 
{
	b3AABB3 aabb;
	aabb.m_lower = b3Min(a.m_lower, b.m_lower);
	aabb.m_upper = b3Max(a.m_upper, b.m_upper);
	return aabb;
}

// Test if two AABBs are overlapping.
inline bool b3TestOverlap(const b3AABB3& a, const b3AABB3& b) 
{
	return (a.m_lower.x <= b.m_upper.x) &&	(a.m_lower.y <= b.m_upper.y) &&	(a.m_lower.z <= b.m_upper.z) &&
		(a.m_upper.x >= b.m_lower.x) && (a.m_upper.y >= b.m_lower.y) &&	(a.m_upper.z >= b.m_lower.z);
}

#endif