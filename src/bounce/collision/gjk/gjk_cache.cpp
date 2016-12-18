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

#include <bounce\collision\gjk\gjk_cache.h>
#include <bounce\collision\gjk\gjk_proxy.h>

extern u32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
u32 b3_gjkCacheHits;

// Implements b3Simplex routines for a cached simplex.
void b3Simplex::ReadCache(const b3SimplexCache* cache, 
	const b3Transform& xfA, const b3GJKProxy& proxyA, 
	const b3Transform& xfB, const b3GJKProxy& proxyB)
{
	B3_ASSERT(cache->count <= 4);
	m_count = (u8)cache->count;
	for (u32 i = 0; i < m_count; ++i)
	{
		b3SimplexVertex* v = m_vertices + i;
		v->indexA = cache->indexA[i];
		v->indexB = cache->indexB[i];
		b3Vec3 wALocal = proxyA.GetVertex(v->indexA);
		b3Vec3 wBLocal = proxyB.GetVertex(v->indexB);
		v->pointA = xfA * wALocal;
		v->pointB = xfB * wBLocal;
		v->point = v->pointB - v->pointA;
		v->weight = 0.0f;
	}

	// Compute the new simplex metric
	// If it is substantially different than
	// old metric then flush the simplex.
	if (m_count > 1)
	{
		float32 metric1 = cache->metric;
		float32 metric2 = GetMetric();
		if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < B3_EPSILON)
		{
			// Flush
			m_count = 0;
		}
		else
		{
			++b3_gjkCacheHits;
		}
	}

	// If cache is empty or flushed choose an arbitrary simplex.
	if (m_count == 0)
	{
		b3SimplexVertex* v = m_vertices + 0;
		b3Vec3 wALocal = proxyA.GetVertex(0);
		b3Vec3 wBLocal = proxyB.GetVertex(0);
		v->pointA = b3Mul(xfA, wALocal);
		v->pointB = b3Mul(xfB, wBLocal);
		v->point = v->pointB - v->pointA;
		v->weight = 1.0f;
		v->indexA = 0;
		v->indexB = 0;
		m_count = 1;
	}
}

void b3Simplex::WriteCache(b3SimplexCache* cache) const
{
	cache->metric = GetMetric();
	cache->count = u16(m_count);
	for (u32 i = 0; i < m_count; ++i)
	{
		cache->indexA[i] = u8(m_vertices[i].indexA);
		cache->indexB[i] = u8(m_vertices[i].indexB);
	}
}

float32 b3Simplex::GetMetric() const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		return 0.0f;
	case 1:
		return 0.0f;
	case 2:
		// Magnitude
		return b3Distance(m_vertices[0].point, m_vertices[1].point);
	case 3:
	{
		// Area
		b3Vec3 E1 = m_vertices[1].point - m_vertices[0].point;
		b3Vec3 E2 = m_vertices[2].point - m_vertices[0].point;
		return b3Length(b3Cross(E1, E2));
	}
	case 4:
	{
		// Volume
		b3Vec3 E1 = m_vertices[1].point - m_vertices[0].point;
		b3Vec3 E2 = m_vertices[2].point - m_vertices[0].point;
		b3Vec3 E3 = m_vertices[3].point - m_vertices[0].point;
		float32 det = b3Det(E1, E2, E3);
		float32 sign = b3Sign(det);
		float32 volume = sign * det;
		return volume;
	}
	default:
		B3_ASSERT(false);
		return 0.0f;
	}
}

b3GJKOutput b3GJK(const b3Transform& xfA, const b3GJKProxy& proxyA,
	const b3Transform& xfB, const b3GJKProxy& proxyB,
	bool applyRadius, b3SimplexCache* cache)
{
	++b3_gjkCalls;

	// Initialize the simplex.
	b3Simplex simplex;
	simplex.ReadCache(cache, xfA, proxyA, xfB, proxyB);

	// Get simplex vertices as an array.
	b3SimplexVertex* vertices = simplex.m_vertices;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	u32 saveA[4], saveB[4];
	u32 saveCount = 0;

	// Last iteration squared distance for checking if we're getting close
	// to the origin and prevent cycling.
	float32 distSq1 = B3_MAX_FLOAT;

	const b3Vec3 kOrigin(0.0f, 0.0f, 0.0f);

	// Limit number of iterations to prevent cycling.
	const u32 kMaxIters = 20;

	// Main iteration loop.
	u32 iter = 0;
	while (iter < kMaxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (u32 i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		// Determine the closest point on the simplex and
		// remove unused vertices.
		switch (simplex.m_count)
		{
		case 1:
			break;
		case 2:
			simplex.Solve2(kOrigin);
			break;
		case 3:
			simplex.Solve3(kOrigin);
			break;
		case 4:
			simplex.Solve4(kOrigin);
			break;
		default:
			B3_ASSERT(false);
			break;
		}

		// If we have 4 points, then the origin is in the corresponding tethrahedron.
		if (simplex.m_count == 4)
		{
			break;
		}

		// Compute the closest point.
		b3Vec3 p = simplex.GetClosestPoint();
		float32 distSq2 = b3Dot(p, p);
		// Ensure we're getting close to the origin.
		if (distSq2 >= distSq1)
		{
			//break;
		}
		distSq1 = distSq2;

		// Get search direction.
		b3Vec3 d = simplex.GetSearchDirection(kOrigin);

		// Ensure the search direction is non-zero.
		if (b3Dot(d, d) < B3_EPSILON * B3_EPSILON)
		{
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b3SimplexVertex* vertex = vertices + simplex.m_count;
		vertex->indexA = proxyA.GetSupportIndex(b3MulT(xfA.rotation, -d));
		vertex->pointA = b3Mul(xfA, proxyA.GetVertex(vertex->indexA));
		vertex->indexB = proxyB.GetSupportIndex(b3MulT(xfB.rotation, d));
		vertex->pointB = b3Mul(xfB, proxyB.GetVertex(vertex->indexB));
		vertex->point = vertex->pointB - vertex->pointA;

		// Iteration count is equated to the number of support point calls.
		++iter;
		++b3_gjkIters;

		// Check for duplicate support points. 
		// This is the main termination criteria.
		bool duplicate = false;
		for (u32 i = 0; i < saveCount; ++i)
		{
			if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++simplex.m_count;
	}

	b3_gjkMaxIters = b3Max(b3_gjkMaxIters, iter);

	// Prepare result.
	b3GJKOutput output;
	simplex.GetClosestPoints(&output.pointA, &output.pointB);
	output.distance = b3Distance(output.pointA, output.pointB);
	output.iterations = iter;

	// Cache the simplex.
	simplex.WriteCache(cache);

	// Apply radius if requested.
	if (applyRadius)
	{
		float32 rA = proxyA.m_radius;
		float32 rB = proxyB.m_radius;

		if (output.distance > rA + rB && output.distance > B3_EPSILON)
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.distance -= rA + rB;
			b3Vec3 d = output.pointB - output.pointA;
			b3Vec3 normal = b3Normalize(d);
			output.pointA += rA * normal;
			output.pointB -= rB * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			b3Vec3 p = 0.5f * (output.pointA + output.pointB);
			output.pointA = p;
			output.pointB = p;
			output.distance = 0.0f;
		}
	}

	// Output result.
	return output;
}
