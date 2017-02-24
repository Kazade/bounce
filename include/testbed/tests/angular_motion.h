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

#ifndef ANGULAR_MOTION_H
#define ANGULAR_MOTION_H

class AngularMotion : public Test
{
public:
	AngularMotion()
	{
		b3BodyDef bdef;
		bdef.type = e_dynamicBody;
		bdef.position.Set(0.0f, 0.0f, 0.0f);

		m_body = m_world.CreateBody(bdef);

		b3CapsuleShape shape;
		shape.m_centers[0].Set(0.0f, 1.0f, 0.0f);
		shape.m_centers[1].Set(0.0f, -1.0f, 0.0f);
		shape.m_radius = 1.0f;

		b3ShapeDef sdef;
		sdef.shape = &shape;
		sdef.density = 1.0f;

		m_body->CreateShape(sdef);

		b3MassData data;
		m_body->GetMassData(&data);

		m_body->SetMassData(&data);

		b3Vec3 g(0.0f, 0.0f, 0.0f);
		m_world.SetGravity(g);
	}

	void Step()
	{
		Test::Step();

		b3Vec3 v(0.0f, 0.0f, 0.0f);
		m_body->SetLinearVelocity(v);

		b3Vec3 p = m_body->GetSweep().worldCenter;
		b3Quat quat = m_body->GetSweep().orientation;

		b3Vec3 axis;
		float32 angle;
		quat.GetAxisAngle(&axis, &angle);
		
		b3Vec3 q(0.0f, 0.0f, 0.0f);
		m_body->SetTransform(q, axis, angle);
	}

	static Test* Create()
	{
		return new AngularMotion();
	}

	b3Body* m_body;
};

#endif