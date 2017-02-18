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

#ifndef CAPSULE_STACK_H
#define CAPSULE_STACK_H

class CapsuleStack : public Test
{
public:
	enum
	{
		e_rowCount = 5,
		e_columnCount = 5,
		e_depthCount = 5
	};

	CapsuleStack()
	{
		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_staticBody;
			
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			
			body->CreateShape(sd);
		}
		
		float32 height = 3.0f;
		float32 radius = 1.0f;
		float32 separation = 0.0f;

		b3CapsuleShape capsule;
		capsule.m_centers[0].Set(0.0f, -0.5f * height, 0.0f);
		capsule.m_centers[1].Set(0.0f, 0.5f * height, 0.0f);
		capsule.m_radius = radius;

		b3ShapeDef sdef;
		sdef.shape = &capsule;
		sdef.density = 1.0f;
		sdef.friction = 0.3f;

		const u32 c = e_rowCount * e_columnCount * e_depthCount;
		b3Body* bs[c];
		u32 n = 0;

		b3AABB3 aabb;
		aabb.m_lower.Set(0.0f, 0.0f, 0.0f);
		aabb.m_upper.Set(0.0f, 0.0f, 0.0f);

		for (u32 i = 0; i < e_rowCount; ++i)
		{
			for (u32 j = 0; j < e_columnCount; ++j)
			{
				for (u32 k = 0; k < e_depthCount; ++k)
				{
					b3BodyDef bdef;
					bdef.type = b3BodyType::e_dynamicBody;
					
					bdef.position.x = (2.0f + separation) * float32(i) * (0.5f * height + radius);
					bdef.position.y = (2.0f + separation) * float32(j) * radius;
					bdef.position.z = (2.0f + separation) * float32(k) * radius;
					
					bdef.orientation = b3Quat(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
					
					b3Body* body = m_world.CreateBody(bdef);
					bs[n++] = body;

					b3Shape* shape = body->CreateShape(sdef);

					b3AABB3 aabb2;
					shape->ComputeAABB(&aabb2, body->GetTransform());

					aabb = b3Combine(aabb, aabb2);
				}
			}
		}

		b3Vec3 center = aabb.Centroid();
		
		for (u32 i = 0; i < n; ++i)
		{
			b3Body* b = bs[i];
			const b3Vec3& p = b->GetSweep().worldCenter;
			const b3Quat& q = b->GetSweep().orientation;

			// centralize
			b3Vec3 position = p - center;
			
			// move up
			position.y += 0.5f * aabb.Height() + radius;

			// maintain orientation
			b->SetTransform(position, q.GetAxis(), q.GetAngle());
		}
	}

	static Test* Create()
	{
		return new CapsuleStack();
	}
};

#endif
