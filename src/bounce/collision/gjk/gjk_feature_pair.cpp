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

#include <bounce/collision/gjk/gjk_cache.h>

b3GJKFeaturePair b3GetFeaturePair(const b3SimplexCache& cache)
{
	B3_ASSERT(0 < cache.count && cache.count < 4);

	u32 vertexCount = cache.count;
	u32 uniqueCount1 = b3UniqueCount(cache.indexA, vertexCount);
	u32 uniqueCount2 = b3UniqueCount(cache.indexB, vertexCount);

	if (vertexCount == 1)
	{
		// VV 
		b3GJKFeaturePair pair;
		pair.typeA = b3GJKFeaturePair::e_vertex;
		pair.typeB = b3GJKFeaturePair::e_vertex;
		pair.indexA[0] = cache.indexA[0];
		pair.indexB[0] = cache.indexB[0];
		return pair;
	}

	if (vertexCount == 2)
	{
		if (uniqueCount1 == 2 && uniqueCount2 == 2)
		{
			// EE
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_edge;
			pair.typeB = b3GJKFeaturePair::e_edge;
			pair.indexA[0] = cache.indexA[0];
			pair.indexA[1] = cache.indexA[1];
			pair.indexB[0] = cache.indexB[0];
			pair.indexB[1] = cache.indexB[1];
			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			return pair;
		}

		if (uniqueCount1 == 1)
		{
			// VE
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_vertex;
			pair.typeB = b3GJKFeaturePair::e_edge;
			pair.indexA[0] = cache.indexA[0];
			pair.indexB[0] = cache.indexB[0];
			pair.indexB[1] = cache.indexB[1];
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			return pair;
		}

		if (uniqueCount2 == 1)
		{
			// EV
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_edge;
			pair.typeB = b3GJKFeaturePair::e_vertex;
			pair.indexA[0] = cache.indexA[0];
			pair.indexA[1] = cache.indexA[1];
			pair.indexB[0] = cache.indexB[0];
			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			return pair;
		}
		B3_ASSERT(false);
	}

	if (vertexCount == 3)
	{
		if (uniqueCount1 == 1 && uniqueCount2 == 3)
		{
			// VF
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_vertex;
			pair.typeB = b3GJKFeaturePair::e_face;
			pair.indexA[0] = cache.indexA[0];
			pair.indexB[0] = cache.indexB[0];
			pair.indexB[1] = cache.indexB[1];
			pair.indexB[2] = cache.indexB[2];
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			B3_ASSERT(pair.indexB[1] != pair.indexB[2]);
			B3_ASSERT(pair.indexB[2] != pair.indexB[0]);
			return pair;
		}

		if (uniqueCount1 == 3 && uniqueCount2 == 1)
		{
			// FV
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_face;
			pair.typeB = b3GJKFeaturePair::e_vertex;
			pair.indexA[0] = cache.indexA[0];
			pair.indexA[1] = cache.indexA[1];
			pair.indexA[2] = cache.indexA[2];
			pair.indexB[0] = cache.indexB[0];
			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			B3_ASSERT(pair.indexA[1] != pair.indexA[2]);
			B3_ASSERT(pair.indexA[2] != pair.indexA[0]);
			return pair;
		}

		if (uniqueCount1 == 2 && uniqueCount2 == 2)
		{
			// EE
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_edge;
			pair.typeB = b3GJKFeaturePair::e_edge;
			pair.indexA[0] = cache.indexA[0];
			pair.indexB[0] = cache.indexB[0];

			if (cache.indexA[0] == cache.indexA[1])
			{
				pair.indexA[1] = cache.indexA[2];
			}
			else
			{
				pair.indexA[1] = cache.indexA[1];
			}

			if (cache.indexB[0] == cache.indexB[1])
			{
				pair.indexB[1] = cache.indexB[2];
			}
			else
			{
				pair.indexB[1] = cache.indexB[1];
			}
			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			return pair;
		}

		if (uniqueCount1 == 2 && uniqueCount2 == 3)
		{
			// EF
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_edge;
			pair.typeB = b3GJKFeaturePair::e_face;

			pair.indexA[0] = cache.indexA[0];
			if (cache.indexA[0] == cache.indexA[1])
			{
				pair.indexA[1] = cache.indexA[2];
			}
			else
			{
				pair.indexA[1] = cache.indexA[1];
			}

			pair.indexB[0] = cache.indexB[0];
			pair.indexB[1] = cache.indexB[1];
			pair.indexB[2] = cache.indexB[2];

			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			B3_ASSERT(pair.indexB[1] != pair.indexB[2]);
			B3_ASSERT(pair.indexB[2] != pair.indexB[0]);
			return pair;
		}

		if (uniqueCount1 == 3 && uniqueCount2 == 2)
		{
			// FE
			b3GJKFeaturePair pair;
			pair.typeA = b3GJKFeaturePair::e_face;
			pair.typeB = b3GJKFeaturePair::e_edge;
			pair.indexA[0] = cache.indexA[0];
			pair.indexA[1] = cache.indexA[1];
			pair.indexA[2] = cache.indexA[2];

			pair.indexB[0] = cache.indexB[0];
			if (cache.indexB[0] == cache.indexB[1])
			{
				pair.indexB[1] = cache.indexB[2];
			}
			else
			{
				pair.indexB[1] = cache.indexB[1];
			}

			B3_ASSERT(pair.indexA[0] != pair.indexA[1]);
			B3_ASSERT(pair.indexA[1] != pair.indexA[2]);
			B3_ASSERT(pair.indexA[2] != pair.indexA[0]);
			B3_ASSERT(pair.indexB[0] != pair.indexB[1]);
			return pair;
		}

		B3_ASSERT(false);
	}

	B3_ASSERT(false);

	b3GJKFeaturePair pair;
	pair.typeA = b3GJKFeaturePair::e_unknown;
	pair.typeB = b3GJKFeaturePair::e_unknown;
	return pair;
}
