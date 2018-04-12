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

#ifndef COMPOUND_BODY_H
#define COMPOUND_BODY_H

class CompoundBody : public Test
{
public:
	CompoundBody()
	{
		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			body->CreateShape(sd);
		}

		{
			b3Transform xf;
			xf.SetIdentity();
			xf.position.Set(-5.0f, 10.0f, 0.0f);
			m_box1.SetTransform(xf);
		}
		
		{
			b3Transform xf;
			xf.SetIdentity();
			xf.position.Set(5.0f, 10.0f, 0.0f);
			m_box2.SetTransform(xf);
		}
		
		{
			b3Transform xf;
			xf.SetIdentity();
			xf.position.Set(0.0f, 2.0f, 0.0f);
			m_box3.SetTransform(xf);
		}
		 
		{
			b3Transform xf;
			xf.SetIdentity();
			xf.position.Set(0.0f, 6.0f, 0.0f);
			m_box4.SetTransform(xf);
		}
		
		{
			b3Transform xf;
			xf.SetIdentity();
			xf.position.Set(0.0f, 10.0f, 0.0f);
			m_box5.SetTransform(xf);
		}
		
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 0.0f, 0.0f);
			bd.angularVelocity.Set(0.0f, 2.0f * B3_PI, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;

			b3ShapeDef sd;
			sd.shape = &hs;
			sd.density = 0.1f;

			hs.m_hull = &m_box1;
			body->CreateShape(sd);
			
			hs.m_hull = &m_box2;
			body->CreateShape(sd);

			hs.m_hull = &m_box3;
			body->CreateShape(sd); 

			hs.m_hull = &m_box4;
			body->CreateShape(sd); 

			hs.m_hull = &m_box5;
			body->CreateShape(sd); 
		}
	}

	static Test* Create()
	{
		return new CompoundBody();
	}

	b3BoxHull m_box1;
	b3BoxHull m_box2;
	b3BoxHull m_box3;
	b3BoxHull m_box4;
	b3BoxHull m_box5;
};

#endif