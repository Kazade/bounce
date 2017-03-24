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

#ifndef THIN_H
#define THIN_H

class Thin : public Test
{
public:
	enum
	{
		e_rowCount = 1,
		e_columnCount = 10,
		e_depthCount = 1
	};

	Thin()
	{
		g_camera.m_center.Set(2.5f, -2.0f, 5.5f);
		g_camera.m_zoom = 40.0f;

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;
			sdef.friction = 1.0f;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		static b3BoxHull thinHull;

		{
			b3Transform xf;
			xf.position.SetZero();
			xf.rotation = b3Diagonal(4.05f, 2.0f * B3_LINEAR_SLOP, 4.05f);
			
			thinHull.SetTransform(xf);
		}

		b3Vec3 stackOrigin;
		stackOrigin.Set(0.0f, 4.05f, 0.0f);

		b3Vec3 boxScale;
		boxScale.Set(4.05f, 2.05f, 4.05f);

		for (u32 i = 0; i < e_rowCount; ++i)
		{
			for (u32 j = 0; j < e_columnCount; ++j)
			{
				for (u32 k = 0; k < e_depthCount; ++k)
				{
					b3BodyDef bdef;
					bdef.type = b3BodyType::e_dynamicBody;

					bdef.position.x = float32(i) * boxScale.x;
					bdef.position.y = 1.5f * float32(j) * boxScale.y;
					bdef.position.z = float32(k) * boxScale.z;
					bdef.position += stackOrigin;

					b3Body* body = m_world.CreateBody(bdef);

					b3HullShape hs;
					hs.m_hull = &thinHull;

					b3ShapeDef sdef;
					sdef.shape = &hs;
					sdef.density = 0.5f;
					sdef.friction = 0.2f;
					
					body->CreateShape(sdef);
				}
			}
		}
	}

	static Test* Create()
	{
		return new Thin();
	}
};

#endif
