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

#ifndef QUADRIC_SHAPES_H
#define QUADRIC_SHAPES_H

class QuadricShapes : public Test
{
public:
	QuadricShapes()
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

		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(-10.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3SphereShape sphere;
			sphere.m_center.SetZero();
			sphere.m_radius = 1.0f;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
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
			bdef.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			m_coneHull.SetAsCone();

			b3HullShape hull;
			hull.m_hull = &m_coneHull;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
			sdef.friction = 0.3f;
			sdef.shape = &hull;

			body->CreateShape(sdef);
		}

		{
			b3BodyDef bdef;
			bdef.type = e_dynamicBody;
			bdef.position.Set(4.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			m_cylinderHull.SetAsCylinder();

			b3HullShape hull;
			hull.m_hull = &m_cylinderHull;

			b3ShapeDef sdef;
			sdef.density = 0.1f;
			sdef.friction = 0.2f;
			sdef.shape = &hull;

			body->CreateShape(sdef);
		}
	}

	~QuadricShapes()
	{
	}

	static Test* Create()
	{
		return new QuadricShapes();
	}

	b3QHull m_coneHull;
	b3QHull m_cylinderHull;
};

#endif