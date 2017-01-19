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

#ifndef B3_GJK_CACHE_H
#define B3_GJK_CACHE_H

#include <bounce/collision/gjk/gjk.h>

// A cached simplex is used to improve the performance 
// of the GJK when called more than once. 
// Make sure to set cache.count to zero before 
// passing this structure as an argument to GJK when called 
// for the first time.
struct b3SimplexCache
{
	float32 metric; // distance or area or volume
	u32 iterations; // number of GJK iterations
	u16 count; // number of support vertices
	u8 indexA[4]; // support vertices on proxy A
	u8 indexB[4]; // support vertices on proxy B
};

// Find the closest points and distance between two proxies. 
// Assumes a simplex is given for increasing the performance of 
// the algorithm when called more than once.
b3GJKOutput b3GJK(const b3Transform& xfA, const b3GJKProxy& proxyA,
				  const b3Transform& xfB, const b3GJKProxy& proxyB,
				  bool applyRadius, b3SimplexCache* cache);

// A feature pair contains the vertices of the features associated 
// with the closest points.
struct b3GJKFeaturePair
{
	u32 indexA[3]; // vertices on proxy A
	u32 countA; // number of vertices on proxy A
	u32 indexB[3]; // vertices on proxy B
	u32 countB; // number of vertices on proxy B
};

// Identify the vertices of the features that the closest points between two 
// GJK proxies are contained on given a cached simplex.
// The GJK must have been called using the pair of proxies and 
// cache.count must be < 4, that is, the proxies must not be overlapping.
b3GJKFeaturePair b3GetFeaturePair(const b3SimplexCache& cache);

#endif