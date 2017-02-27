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

#ifndef SPRING_H
#define SPRING_H

class Spring : public Test
{
public:
	Spring()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;
			
			ground->CreateShape(sd);
		}
		
		// Car frame shape
		{
			b3Transform xf;
			xf.SetIdentity();
			xf.rotation = b3Diagonal(2.0f, 0.5f, 5.0f);

			m_frameHull.SetTransform(xf);
		}

		b3HullShape box;
		box.m_hull = &m_frameHull;

		// Wheel shape
		b3SphereShape sphere;
		sphere.m_center.SetZero();
		sphere.m_radius = 1.0f;

		// Car frame
		b3Body* frame;
		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(0.0f, 10.0f, 0.0f);
			
			frame = m_world.CreateBody(bdef);

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 0.3f;
			sdef.shape = &box;
			
			frame->CreateShape(sdef);
		}
		
		b3Body* wheelLF;
		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(-1.0f, 7.0f, -4.5f);
			bdef.fixedRotationY = true;

			wheelLF = m_world.CreateBody(bdef);

			b3ShapeDef sdef;
			sdef.shape = &sphere;
			sdef.density = 1.0f;
			sdef.friction = 1.0f;
			
			wheelLF->CreateShape(sdef);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelLF, b3Vec3(-1.0f, 9.0f, -4.5), b3Vec3(-1.0f, 9.0f, -4.5f));
			def.collideLinked = true;
			def.dampingRatio = 0.5f;
			def.frequencyHz = 4.0f;
			
			m_world.CreateJoint(def);
		}

		b3Body* wheelRF;
		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(1.0f, 7.0, -4.5f);
			bdef.fixedRotationY = true;

			wheelRF = m_world.CreateBody(bdef);

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 1.0f;
			sdef.shape = &sphere;
			
			wheelRF->CreateShape(sdef);
		}
		
		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelRF, b3Vec3(1.0f, 9.0, -4.5), b3Vec3(1.0f, 9.0, -4.5f));
			def.collideLinked = true;
			def.dampingRatio = 0.5f;
			def.frequencyHz = 4.0f;

			m_world.CreateJoint(def);
		}
		
		b3Body* wheelLB;
		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(-1.0f, 7.0f, 4.5f);
			bdef.fixedRotationY = true;

			wheelLB = m_world.CreateBody(bdef);

			b3ShapeDef sdef;
			sdef.shape = &sphere;
			sdef.density = 1.0f;
			sdef.friction = 1.0f;

			wheelLB->CreateShape(sdef);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelLB, b3Vec3(-1.0f, 9.0f, 4.5f), b3Vec3(-1.0f, 9.0f, 4.5f));
			def.collideLinked = true;
			def.dampingRatio = 0.8f;
			def.frequencyHz = 4.0f;

			m_world.CreateJoint(def);
		}

		b3Body* wheelRB;
		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(1.0f, 7.0f, 4.5f);
			bdef.fixedRotationY = true;

			wheelRB = m_world.CreateBody(bdef);

			b3ShapeDef sdef;
			sdef.density = 1.0f;
			sdef.friction = 1.0f;
			sdef.shape = &sphere;

			wheelRB->CreateShape(sdef);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelRB, b3Vec3(1.0f, 9.0f, 4.5f), b3Vec3(1.0f, 9.0f, 4.5f));
			def.collideLinked = true;
			def.frequencyHz = 4.0f;
			def.dampingRatio = 0.8f;

			m_world.CreateJoint(def);
		}
	}

	static Test* Create()
	{
		return new Spring();
	}

	b3BoxHull m_frameHull;
};

#endif