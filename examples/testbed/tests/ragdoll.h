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

#ifndef RAGDOLL_H
#define RAGDOLL_H

class Ragdoll : public Test
{
public:
	Ragdoll()
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

		b3Body* head;
		b3Body* hip;
		b3Body* lArm;
		b3Body* rArm;
		b3Body* lLeg;
		b3Body* rLeg;

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 10.0f, 0.0f);
			hip = m_world.CreateBody(bd);

			hip->ApplyForceToCenter(b3Vec3(0.0f, 0.0f, -5000.0f), true);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 0.5f, 0.0f);
			cs.m_centers[1].Set(0.0f, -0.5f, 0.0f);
			cs.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 1.0f;
			hip->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 12.25f, 0.0f);
			bd.angularVelocity.Set(0.0f, 0.0f, 0.005f);
			head = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 0.15f, 0.0f);
			cs.m_centers[1].Set(0.0f, -0.15f, 0.0f);
			cs.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 5.0f;
			head->CreateShape(sd);
		}

		// Link head to chest
		{
			b3ConeJointDef cd;
			cd.bodyA = hip;
			cd.bodyB = head;
			cd.collideLinked = false;
			cd.enableLimit = true;
			cd.Initialize(hip, head, b3Vec3(0.0f, 1.0f, 0.0f), b3Vec3(0.0f, 11.55f, 0.0f), 0.25f * B3_PI);
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-2.5f, 11.0f, 0.0f);
			bd.orientation.Set(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
			lArm = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 1.0f, 0.0f);
			cs.m_centers[1].Set(0.0f, -1.0f, 0.0f);
			cs.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 0.25f;

			lArm->CreateShape(sd);
		}

		// Link left arm to chest
		{
			b3ConeJointDef cd;
			cd.bodyA = hip;
			cd.bodyB = lArm;
			cd.collideLinked = false;
			cd.enableLimit = true;
			cd.Initialize(hip, lArm, b3Vec3(-1.0f, 0.0f, 0.0f), b3Vec3(-1.0f, 11.0f, 0.0f), B3_PI);
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(2.5f, 11.0f, 0.0f);
			bd.orientation.Set(b3Vec3(0.0f, 0.0f, 1.0f), 0.5f * B3_PI);
			rArm = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 1.0f, 0.0f);
			cs.m_centers[1].Set(0.0f, -1.0f, 0.0f);
			cs.m_radius = 0.5f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 0.25f;

			rArm->CreateShape(sd);
		}

		// Link right arm to chest
		{
			b3ConeJointDef cd;
			cd.bodyA = hip;
			cd.bodyB = rArm;
			cd.collideLinked = false;
			cd.enableLimit = true;
			cd.Initialize(hip, rArm, b3Vec3(1.0f, 0.0f, 0.0f), b3Vec3(1.0f, 11.0f, 0.0f), B3_PI);
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-0.5f, 6.0f, 0.0f);
			lLeg = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 2.0f, 0.0f);
			cs.m_centers[1].Set(0.0f, -2.0f, 0.0f);
			cs.m_radius = 0.45f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 0.25f;

			lLeg->CreateShape(sd);
		}

		// Link left leg to chest
		{
			b3ConeJointDef cd;
			cd.bodyA = hip;
			cd.bodyB = lLeg;
			cd.collideLinked = false;
			cd.enableLimit = true;
			cd.Initialize(hip, lLeg, b3Vec3(0.0f, -1.0f, 0.0f), b3Vec3(-0.5f, 8.5f, 0.0f), 0.25f * B3_PI);
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.5f, 6.0f, 0.0f);
			rLeg = m_world.CreateBody(bd);

			b3CapsuleShape cs;
			cs.m_centers[0].Set(0.0f, 2.0f, 0.0f);
			cs.m_centers[1].Set(0.0f, -2.0f, 0.0f);
			cs.m_radius = 0.45f;

			b3ShapeDef sd;
			sd.shape = &cs;
			sd.density = 0.25f;

			rLeg->CreateShape(sd);
		}

		// Link right leg to chest
		{
			b3ConeJointDef cd;
			cd.bodyA = hip;
			cd.bodyB = rLeg;
			cd.collideLinked = false;
			cd.enableLimit = true;
			cd.Initialize(hip, rLeg, b3Vec3(0.0f, -1.0f, 0.0f), b3Vec3(0.5f, 8.5f, 0.0f), 0.25f * B3_PI);
			b3ConeJoint* cj = (b3ConeJoint*)m_world.CreateJoint(cd);
		}
	}

	void KeyDown(int button)
	{
	}

	static Test* Create()
	{
		return new Ragdoll();
	}
};

#endif