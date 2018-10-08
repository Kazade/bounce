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

	b3RectangleGarment(float32 ex = 1.0f, float32 ey = 1.0f) : rectangle(ex, ey)
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

	b3CircleGarment(float32 r = 1.0f) : circle(r)
	{
		circlePatterns[0] = &circle;

		patternCount = 1;
		patterns = circlePatterns;

		sewingCount = 0;
		sewingLines = nullptr;
	}
};

struct b3ShirtGarment : public b3Garment
{
	b3RectanglePattern frontBody;
	b3RectanglePattern frontRightSleeve;
	b3RectanglePattern frontLeftSleeve;
	
	b3RectanglePattern backBody;
	b3RectanglePattern backRightSleeve;
	b3RectanglePattern backLeftSleeve;

	b3SewingLine shirtSewingLines[12];

	b3SewingPattern* shirtPatterns[6];

	b3ShirtGarment() : 
		frontBody(1.0f, 1.0f),
		frontRightSleeve(0.2f, 0.2f),
		frontLeftSleeve(0.2f, 0.2f),
		backBody(1.0f, 1.0f),
		backRightSleeve(0.2f, 0.2f),
		backLeftSleeve(0.2f, 0.2f)
	{
		b3Vec2 etr;
		etr.x = 1.2f;
		etr.y = 0.8f;

		b3Vec2 etl;
		etl.x = -1.2f;
		etl.y = 0.8f;
		
		for (u32 i = 0; i < 4; ++i)
		{
			frontRightSleeve.vertices[i] += etr;
			frontLeftSleeve.vertices[i] += etl;
			backRightSleeve.vertices[i] += etr;
			backLeftSleeve.vertices[i] += etl;
		}
		
		// Perform sewing
		u32 count = 0;
		
		// Sew bodies
		for (u32 i = 0; i < frontBody.vertexCount; ++i)
		{
			b3SewingLine line;
			line.p1 = 0;
			line.p2 = 3;
			line.v1 = i;
			line.v2 = i;

			shirtSewingLines[count++] = line;
		}

		// Sew sleeves
		for (u32 i = 0; i < frontRightSleeve.vertexCount; ++i)
		{
			b3SewingLine line;
			line.p1 = 1;
			line.p2 = 4;
			line.v1 = i;
			line.v2 = i;

			shirtSewingLines[count++] = line;
		}

		for (u32 i = 0; i < frontLeftSleeve.vertexCount; ++i)
		{
			b3SewingLine line;
			line.p1 = 2;
			line.p2 = 5;
			line.v1 = i;
			line.v2 = i;

			shirtSewingLines[count++] = line;
		}

		B3_ASSERT(12 == count);

		shirtPatterns[0] = &frontBody;
		shirtPatterns[1] = &frontRightSleeve;
		shirtPatterns[2] = &frontLeftSleeve;

		shirtPatterns[3] = &backBody;
		shirtPatterns[4] = &backRightSleeve;
		shirtPatterns[5] = &backLeftSleeve;

		patternCount = 6;
		patterns = shirtPatterns;

		sewingCount = 12;
		sewingLines = shirtSewingLines;
	}
};

#endif