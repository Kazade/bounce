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

extern Settings g_settings;

class RayCast : public Test
{
public:
	RayCast()
	{
		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(0.0f, 2.0f, 10.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(-10.0f, 6.0f, -10.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_tallHull;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(10.0f, 2.0f, 0.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.20f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(-10.0f, 2.0f, 14.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), 0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(-14.0f, 2.0f, 5.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), -0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(20.0f, 2.0f, 5.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), -0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}
		
		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(12.0f, 2.0f, 5.0f);
			bdef.orientation = b3Quat(b3Vec3(0.0f, 1.0f, 0.0f), -0.35f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3SphereShape hs;
			hs.m_radius = 2.5f;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = b3BodyType::e_staticBody;
			bdef.position.Set(0.0f, 1.0f, -12.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3CapsuleShape hs;
			hs.m_centers[0].Set(0.0f, 1.0f, 0.0f);
			hs.m_centers[1].Set(0.0f, -1.0f, 0.0f);
			hs.m_radius = 3.0f;

			b3ShapeDef sdef;
			sdef.density = 0.0f;
			sdef.friction = 0.0f;
			sdef.shape = &hs;
			sdef.userData = NULL;

			b3Shape* shape = body->CreateShape(sdef);
		}

		m_p1.Set(0.0f, 2.0f, 0.0f);
		m_p2.Set(50.0f, 2.0f, 0.0f);

		m_p12.Set(0.0f, 2.0f, 0.0f);
		m_p22.Set(-50.0f, 2.0f, 0.0f);
	}

	void CastRay(const b3Vec3 p1, const b3Vec3 p2) const
	{
		// Perform the ray cast
		RayCastListener listener;
		listener.hit.shape = NULL;
		m_world.RayCastFirst(&listener, p1, p2);

		RayCastHit hit = listener.hit;
		if (hit.shape)
		{
			// Replace current hit
			g_debugDraw->DrawSegment(p1, hit.point, b3Color(0.0f, 1.0f, 0.0f));
			g_debugDraw->DrawPoint(hit.point, 4.0f, b3Color(1.0f, 0.0f, 0.0f));
			g_debugDraw->DrawSegment(hit.point, hit.point + hit.normal, b3Color(1.0f, 1.0f, 1.0f));
		}
		else
		{
			g_debugDraw->DrawSegment(p1, p2, b3Color(0.0f, 1.0f, 0.0f));
		}
	}

	void Step()
	{
		float32 dt = g_settings.hertz > 0.0f ? 1.0f / g_settings.hertz : 0.0f;
		b3Quat q(b3Vec3(0.0f, 1.0f, 0.0f), dt * 0.05f * B3_PI);
		
		m_p1 = b3Mul(q, m_p1);
		m_p2 = b3Mul(q, m_p2);

		m_p12 = b3Mul(q, m_p12);
		m_p22 = b3Mul(q, m_p22);

		CastRay(m_p1, m_p2);
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
