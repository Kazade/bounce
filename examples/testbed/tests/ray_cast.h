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

#ifndef RAY_CAST_H
#define RAY_CAST_H

class RayCast : public Test
{
public:
	RayCast()
	{
		{
			b3BodyDef bdef;
			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(0.0f, 2.0f, 10.0f);
			bdef.orientation = b3QuatRotationY(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 6.0f, -10.0f);
			bdef.orientation = b3QuatRotationY(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);
			
			static b3BoxHull boxHull(2.0f, 4.0f, 0.5f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(10.0f, 2.0f, 0.0f);
			bdef.orientation = b3QuatRotationY(0.20f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 2.0f, 14.0f);
			bdef.orientation = b3QuatRotationY(0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(-14.0f, 2.0f, 5.0f);
			bdef.orientation = b3QuatRotationY(-0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(20.0f, 2.0f, 5.0f);
			bdef.orientation = b3QuatRotationY(-0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.position.Set(12.0f, 2.0f, 5.0f);
			bdef.orientation = b3QuatRotationY(-0.35f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3SphereShape hs;
			hs.m_center.SetZero();
			hs.m_radius = 2.5f;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.position.Set(0.0f, 1.0f, -12.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3CapsuleShape hs;
			hs.m_centers[0].Set(0.0f, 1.0f, 0.0f);
			hs.m_centers[1].Set(0.0f, -1.0f, 0.0f);
			hs.m_radius = 3.0f;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		m_p1.Set(0.0f, 2.0f, 0.0f);
		m_p2.Set(50.0f, 2.0f, 0.0f);

		m_p12.Set(0.0f, 2.0f, 0.0f);
		m_p22.Set(-50.0f, 2.0f, 0.0f);
	}

	void CastRay(const b3Vec3 p1, const b3Vec3 p2) const
	{
		b3ShapeRayCastSingleOutput out;
		if (m_world.RayCastSingleShape(&out, p1, p2))
		{
			g_draw->DrawSegment(p1, out.point, b3Color_green);
			
			g_draw->DrawPoint(out.point, 4.0f, b3Color_red);
			g_draw->DrawSegment(out.point, out.point + out.normal, b3Color_white);
		}
		else
		{
			g_draw->DrawSegment(p1, p2, b3Color_green);
		}
	}

	void Step()
	{
		float32 dt = g_testSettings->inv_hertz;
		b3Quat dq = b3QuatRotationY(0.05f * B3_PI * dt);
		
		m_p1 = b3Mul(dq, m_p1);
		m_p2 = b3Mul(dq, m_p2);
		CastRay(m_p1, m_p2);

		m_p12 = b3Mul(dq, m_p12);
		m_p22 = b3Mul(dq, m_p22);
		CastRay(m_p12, m_p22);

		Test::Step();
	}

	static Test* Create()
	{
		return new RayCast();
	}

	b3Vec3 m_p1, m_p2;
	b3Vec3 m_p12, m_p22;
};

#endif