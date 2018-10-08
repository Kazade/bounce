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

#ifndef B3_SEWING_PATTERN_H
#define B3_SEWING_PATTERN_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/mat22.h>

struct b3SewingPattern
{
	u32 vertexCount;
	b3Vec2* vertices;
};

struct b3RectanglePattern : public b3SewingPattern
{
	b3Vec2 rectangleVertices[4];

	b3RectanglePattern(float32 ex = 1.0f, float32 ey = 1.0f)
	{
		B3_ASSERT(ex > 0.0f);
		B3_ASSERT(ey > 0.0f);

		// R, CCW
		rectangleVertices[0].Set(ex, -ey);
		rectangleVertices[1].Set(ex, ey);

		// L, CW
		rectangleVertices[2].Set(-ex, ey);
		rectangleVertices[3].Set(-ex, -ey);

		vertexCount = 4;
		vertices = rectangleVertices;
	}
};

template<u32 E = 16>
struct b3CirclePattern : public b3SewingPattern
{
	b3Vec2 circleVertices[E];

	b3CirclePattern(float32 radius = 1.0f)
	{
		b3Vec2 center = b3Vec2_zero;

		float32 x = 2.0f * B3_PI / float32(E);
		float32 c = cos(x);
		float32 s = sin(x);

		b3Mat22 R;
		R.x.Set(c, s);
		R.y.Set(-s, c);

		b3Vec2 n(1.0f, 0.0f);
		circleVertices[0] = center + radius * n;
		for (u32 i = 1; i < E; ++i)
		{
			b3Vec2 ni = R * n;
			circleVertices[i] = center + radius * ni;
			n = ni;
		}

		vertexCount = E;
		vertices = circleVertices;
	}
};

#endif