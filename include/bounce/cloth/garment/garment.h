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

#ifndef B3_GARMENT_H
#define B3_GARMENT_H

#include <bounce/cloth/garment/sewing_pattern.h>

struct b3SewingLine
{
	u32 p1, p2;
	u32 v1, v2;
};

struct b3Garment
{
	u32 patternCount;
	b3SewingPattern** patterns;
	u32 sewingCount;
	b3SewingLine* sewingLines;
};

struct b3RectangleGarment : public b3Garment
{
	b3RectanglePattern rectangle;
	b3SewingPattern* rectanglePatterns[1];

	b3RectangleGarment(scalar ex = scalar(1), scalar ey = scalar(1)) : rectangle(ex, ey)
	{
		rectanglePatterns[0] = &rectangle;

		patternCount = 1;
		patterns = rectanglePatterns;

		sewingCount = 0;
		sewingLines = nullptr;
	}
};

struct b3CircleGarment : public b3Garment
{
	b3CirclePattern<> circle;
	b3SewingPattern* circlePatterns[1];

	b3CircleGarment(scalar r = scalar(1)) : circle(r)
	{
		circlePatterns[0] = &circle;

		patternCount = 1;
		patterns = circlePatterns;

		sewingCount = 0;
		sewingLines = nullptr;
	}
};

#endif