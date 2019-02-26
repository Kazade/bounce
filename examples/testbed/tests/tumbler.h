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

#ifndef TUMBLER_TEST_H
#define TUMBLER_TEST_H

class Tumbler : public Test
{
public:
	enum
	{
		e_count = 100
	};

	Tumbler()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			bd.type = e_dynamicBody;
			b3Body* rotor = m_world.CreateBody(bd);

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(0.0f, -45.0f, 0.0f);
				m.rotation = b3Diagonal(50.0f, 1.0f, 200.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(0.0f, 50.0f, 0.0f);
				m.rotation = b3Diagonal(50.0f, 1.0f, 200.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(0.0f, 5.0f, -200.0f);
				m.rotation = b3Diagonal(50.0f, 50.0f, 1.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(0.0f, 5.0f, 200.0f);
				m.rotation = b3Diagonal(50.0f, 50.0f, 1.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(-50.0f, 5.0f, 0.0f);
				m.rotation = b3Diagonal(1.0f, 50.0f, 200.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				static b3BoxHull box;

				b3Transform m;
				m.position.Set(50.0f, 5.0f, 0.0f);
				m.rotation = b3Diagonal(1.0f, 50.0f, 200.0f);

				box.SetTransform(m);

				b3HullShape hs;
				hs.m_hull = &box;

				b3ShapeDef sd;
				sd.density = 5.0f;
				sd.shape = &hs;

				rotor->CreateShape(sd);
			}

			{
				b3RevoluteJointDef jd;
				jd.Initialize(ground, rotor, b3Vec3(0.0f, 0.0f, -1.0f), ground->GetPosition(), -B3_PI, B3_PI);
				jd.motorSpeed = 0.05f * B3_PI;
				jd.maxMotorTorque = 1000.0f * rotor->GetMass();
				jd.enableMotor = true;
				
				b3Joint* joint = (b3RevoluteJoint*)m_world.CreateJoint(jd);
			}
		}

		m_coneHull.SetAsCone();
		m_cylinderHull.SetAsCylinder();

		m_count = 0;
	}

	~Tumbler()
	{
	}

	void Step()
	{
		if(m_count < e_count)
		{
			++m_count;

			{
				b3BodyDef bdef;
				bdef.type = e_dynamicBody;
				bdef.position.Set(-10.0f, 5.0f, 0.0f);

				b3Body* body = m_world.CreateBody(bdef);

				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3ShapeDef sdef;
				sdef.density = 1.0f;
				sdef.friction = 0.3f;
				sdef.shape = &sphere;

				body->CreateShape(sdef);
			}

			{
				b3BodyDef bdef;
				bdef.type = e_dynamicBody;
				bdef.position.Set(-5.0f, 5.0f, 0.0f);

				b3Body* body = m_world.CreateBody(bdef);

				b3CapsuleShape capsule;
				capsule.m_centers[0].Set(0.0f, 0.0f, -1.0f);
				capsule.m_centers[1].Set(0.0f, 0.0f, 1.0f);
				capsule.m_radius = 1.0f;

				b3ShapeDef sdef;
				sdef.density = 0.1f;
				sdef.friction = 0.2f;
				sdef.shape = &capsule;

				body->CreateShape(sdef);
			}

			{
				b3BodyDef bdef;
				bdef.type = e_dynamicBody;
				bdef.position.Set(0.0f, 0.0f, 0.0f);
				bdef.angularVelocity.Set(0.0f, 0.05f * B3_PI, 0.0f);
				b3Body* body = m_world.CreateBody(bdef);

				b3HullShape hs;
				hs.m_hull = &b3BoxHull_identity;

				b3ShapeDef sd;
				sd.density = 0.05f;
				sd.shape = &hs;

				body->CreateShape(sd);
			}

			{
				b3BodyDef bdef;
				bdef.type = e_dynamicBody;
				bdef.position.Set(0.0f, 5.0f, 0.0f);

				b3Body* body = m_world.CreateBody(bdef);

				b3HullShape hull;
				hull.m_hull = &m_coneHull;

				b3ShapeDef sdef;
				sdef.density = 1.0f;
				sdef.friction = 0.3f;
				sdef.shape = &hull;

				body->CreateShape(sdef);
			}

			{
				b3BodyDef bdef;
				bdef.type = e_dynamicBody;
				bdef.position.Set(4.0f, 5.0f, 0.0f);

				b3Body* body = m_world.CreateBody(bdef);

				b3HullShape hull;
				hull.m_hull = &m_cylinderHull;

				b3ShapeDef sdef;
				sdef.density = 1.0f;
				sdef.friction = 0.2f;
				sdef.shape = &hull;

				body->CreateShape(sdef);
			}
		}

		Test::Step();
	}

	static Test* Create()
	{
		return new Tumbler();
	}

	u32 m_count;
	b3QHull m_coneHull;
	b3QHull m_cylinderHull;
};

#endif