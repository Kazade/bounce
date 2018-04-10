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

#ifndef HINGE_CHAIN_H
#define HINGE_CHAIN_H

class HingeChain : public Test
{
public:
	HingeChain()
	{
		static b3BoxHull doorHull;
		{
			b3Transform xf;
			xf.position.SetZero();
			xf.rotation = b3Diagonal(2.0f, 4.0f, 0.5f);
			doorHull.SetTransform(xf);
		}

		float32 x = -50.0f;
		float32 y = 0.0f;

		b3Body* lastHinge;
		{
			b3BodyDef bd;
			bd.position.Set(x, y, 0.0f);
			lastHinge = m_world.CreateBody(bd);

			b3HullShape hull;
			hull.m_hull = &doorHull;

			b3ShapeDef sdef;
			sdef.shape = &hull;
			
			lastHinge->CreateShape(sdef);
		}
		
		x += 4.25f;

		for (u32 i = 0; i < 20; ++i) 
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(x, y, 0.0f);
			b3Body* hinge = m_world.CreateBody(bd);

			b3HullShape hull;
			hull.m_hull = &doorHull;
			
			b3ShapeDef sdef;
			sdef.shape = &hull;
			sdef.density = 1.0f;

			hinge->CreateShape(sdef);
			
			{
				b3Vec3 hingeAxis(0.0f, 1.0f, 0.0f);
				b3Vec3 hingeAnchor(x - 2.25f, y, 0.0f);

				b3RevoluteJointDef jd;
				jd.Initialize(lastHinge, hinge, hingeAxis, hingeAnchor, 0.0f, 0.5f * B3_PI);
				jd.collideLinked = false;

				b3RevoluteJoint* rj = (b3RevoluteJoint*)m_world.CreateJoint(jd);
			} 
			
			x += 4.25f;
			lastHinge = hinge;
		}
	}

	static Test* Create()
	{
		return new HingeChain();
	}
};

#endif
