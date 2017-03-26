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

#include <bounce/collision/gjk/gjk.h>

b3GJKFeaturePair b3GetFeaturePair(const b3SimplexCache& cache)
{
	B3_ASSERT(0 < cache.count && cache.count < 4);

	u32 vertexCount = cache.count;
	u32 uniqueCount1 = b3UniqueCount(cache.index1, vertexCount);
	u32 uniqueCount2 = b3UniqueCount(cache.index2, vertexCount);

	if (vertexCount == 1)
	{
		// VV 
		b3GJKFeaturePair pair;
		pair.count1 = 1;
		pair.count2 = 1;
		pair.index1[0] = cache.index1[0];
		pair.index2[0] = cache.index2[0];
		return pair;
	}

	if (vertexCount == 2)
	{
		if (uniqueCount1 == 2 && uniqueCount2 == 2)
		{
			// EE
			b3GJKFeaturePair pair;
			pair.count1 = 2;
			pair.count2 = 2;
			pair.index1[0] = cache.index1[0];
			pair.index1[1] = cache.index1[1];
			pair.index2[0] = cache.index2[0];
			pair.index2[1] = cache.index2[1];
			B3_ASSERT(pair.index1[0] != pair.index1[1]);
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			return pair;
		}

		if (uniqueCount1 == 1)
		{
			// VE
			b3GJKFeaturePair pair;
			pair.count1 = 1;
			pair.count2 = 2;
			pair.index1[0] = cache.index1[0];
			pair.index2[0] = cache.index2[0];
			pair.index2[1] = cache.index2[1];
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			return pair;
		}

		if (uniqueCount2 == 1)
		{
			// EV
			b3GJKFeaturePair pair;
			pair.count1 = 2;
			pair.count2 = 1;
			pair.index1[0] = cache.index1[0];
			pair.index1[1] = cache.index1[1];
			pair.index2[0] = cache.index2[0];
			B3_ASSERT(pair.index1[0] != pair.index1[1]);
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
			pair.count1 = 1;
			pair.count2 = 3;
			pair.index1[0] = cache.index1[0];
			pair.index2[0] = cache.index2[0];
			pair.index2[1] = cache.index2[1];
			pair.index2[2] = cache.index2[2];
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			B3_ASSERT(pair.index2[1] != pair.index2[2]);
			B3_ASSERT(pair.index2[2] != pair.index2[0]);
			return pair;
		}

		if (uniqueCount1 == 3 && uniqueCount2 == 1)
		{
			// FV
			b3GJKFeaturePair pair;
			pair.count1 = 3;
			pair.count2 = 1;
			pair.index1[0] = cache.index1[0];
			pair.index1[1] = cache.index1[1];
			pair.index1[2] = cache.index1[2];
			pair.index2[0] = cache.index2[0];
			B3_ASSERT(pair.index1[0] != pair.index1[1]);
			B3_ASSERT(pair.index1[1] != pair.index1[2]);
			B3_ASSERT(pair.index1[2] != pair.index1[0]);
			return pair;
		}

		if (uniqueCount1 == 2 && uniqueCount2 == 2)
		{
			// EE
			b3GJKFeaturePair pair;
			pair.count1 = 2;
			pair.count2 = 2;
			pair.index1[0] = cache.index1[0];
			pair.index2[0] = cache.index2[0];

			if (cache.index1[0] == cache.index1[1])
			{
				pair.index1[1] = cache.index1[2];
			}
			else
			{
				pair.index1[1] = cache.index1[1];
			}

			if (cache.index2[0] == cache.index2[1])
			{
				pair.index2[1] = cache.index2[2];
			}
			else
			{
				pair.index2[1] = cache.index2[1];
			}
			B3_ASSERT(pair.index1[0] != pair.index1[1]);
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			return pair;
		}

		if (uniqueCount1 == 2 && uniqueCount2 == 3)
		{
			// EF
			b3GJKFeaturePair pair;
			pair.count1 = 2;
			pair.count2 = 3;

			pair.index1[0] = cache.index1[0];
			if (cache.index1[0] == cache.index1[1])
			{
				pair.index1[1] = cache.index1[2];
			}
			else
			{
				pair.index1[1] = cache.index1[1];
			}

			pair.index2[0] = cache.index2[0];
			pair.index2[1] = cache.index2[1];
			pair.index2[2] = cache.index2[2];

			B3_ASSERT(pair.index1[0] != pair.index1[1]);
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			B3_ASSERT(pair.index2[1] != pair.index2[2]);
			B3_ASSERT(pair.index2[2] != pair.index2[0]);
			return pair;
		}

		if (uniqueCount1 == 3 && uniqueCount2 == 2)
		{
			// FE
			b3GJKFeaturePair pair;
			pair.count1 = 3;
			pair.count2 = 2;
			pair.index1[0] = cache.index1[0];
			pair.index1[1] = cache.index1[1];
			pair.index1[2] = cache.index1[2];

			pair.index2[0] = cache.index2[0];
			if (cache.index2[0] == cache.index2[1])
			{
				pair.index2[1] = cache.index2[2];
			}
			else
			{
				pair.index2[1] = cache.index2[1];
			}

			B3_ASSERT(pair.index1[0] != pair.index1[1]);
			B3_ASSERT(pair.index1[1] != pair.index1[2]);
			B3_ASSERT(pair.index1[2] != pair.index1[0]);
			B3_ASSERT(pair.index2[0] != pair.index2[1]);
			return pair;
		}

		B3_ASSERT(false);
	}

	B3_ASSERT(false);

	b3GJKFeaturePair pair;
	pair.count1 = 0;
	pair.count2 = 0;
	return pair;
}
