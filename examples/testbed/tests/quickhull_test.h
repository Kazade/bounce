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

#ifndef QHULL_H
#define QHULL_H

#include <bounce/quickhull/qh_hull.h>

class QuickhullTest : public Test
{
public:
	QuickhullTest()
	{
		b3BoxHull box;
		box.SetIdentity();

		b3StackArray<b3Vec3, 256> tetra;
		b3Vec3 v1(-1.0f, 0.0f, 0.0f);
		b3Vec3 v2(1.0f, 0.0f, 0.0f);
		b3Vec3 v3(0.0f, 0.0f, -1.0f);
		b3Vec3 v4 = 0.5f * (v1 + v2 + v3);
		v4.y += 2.0f;

		tetra.PushBack(v1);
		tetra.PushBack(v2);
		tetra.PushBack(v3);
		tetra.PushBack(v4);

		// Minkowski sum of box and tetrahedron
		b3StackArray<b3Vec3, 256> points;
		for (u32 i = 0; i < box.vertexCount; ++i)
		{
			for (u32 j = 0; j < tetra.Count(); ++j)
			{
				b3Vec3 p = box.vertices[i] - tetra[j];
				points.PushBack(p);
			}
		}

		u32 size = qhGetMemorySize(points.Count());
		m_memory = b3Alloc(size);
		m_qhull.Construct(m_memory, points);
	}

	~QuickhullTest()
	{
		b3Free(m_memory);
	}

	void Step()
	{
		g_draw->DrawString(b3Color_white, "Iterations = %d", m_qhull.GetIterations());
		m_qhull.Draw();
	}

	static Test* Create()
	{
		return new QuickhullTest();
	}

	void* m_memory;
	qhHull m_qhull;
};

#endif