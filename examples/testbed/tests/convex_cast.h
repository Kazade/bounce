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

#ifndef CONVEX_CAST_H
#define CONVEX_CAST_H

class ConvexCast : public Test
{
public:
	ConvexCast()
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
			bdef.position.Set(0.0f, 2.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bdef);

			b3HullShape hs;
			hs.m_hull = &b3BoxHull_identity;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			m_shape = body->CreateShape(sdef);
		}
		
		{
			m_grid.BuildTree();
			m_grid.BuildAdjacency();

			b3BodyDef bdef;
			bdef.position.Set(-10.0f, 5.0f, -2.0f);
			bdef.orientation = b3QuatRotationZ(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bdef);

			b3MeshShape hs;
			hs.m_mesh = &m_grid;
			hs.m_scale.Set(-1.0f, -1.0f, -2.0f);

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
			hs.m_vertex1.Set(0.0f, 1.0f, 0.0f);
			hs.m_vertex2.Set(0.0f, -1.0f, 0.0f);
			hs.m_radius = 3.0f;

			b3ShapeDef sdef;
			sdef.shape = &hs;

			b3Shape* shape = body->CreateShape(sdef);
		}

		m_d.Set(-50.0f, 2.0f, 0.0f);
	}

	void CastConvex() const
	{
		class ConvexCastFilter : public b3ConvexCastFilter
		{
		public:
			bool ShouldConvexCast(b3Shape* shape)
			{
				return true;
			}
		};

		ConvexCastFilter filter;
		
		b3Body* body = m_shape->GetBody();
		b3Transform xf = body->GetTransform();

		m_world.DrawSolidShape(xf, m_shape, b3Color_red);

		b3Vec3 p1 = xf.translation;

		b3Vec3 p2 = p1 + m_d;
		
		b3ConvexCastSingleOutput out;
		if (m_world.ConvexCastSingle(&out, &filter, m_shape, m_d))
		{
			g_draw->DrawPoint(out.point, 4.0f, b3Color_red);
			g_draw->DrawSegment(out.point, out.point + out.normal, b3Color_white);
			
			b3Transform xft;
			xft.rotation = xf.rotation;
			xft.translation = xf.translation + out.fraction * m_d;

			m_world.DrawShape(xft, m_shape, b3Color_red);

			g_draw->DrawSegment(p1, xft.translation, b3Color_green);
		}
		else
		{
			g_draw->DrawSegment(p1, p2, b3Color_green);
			
			b3Transform xf1;
			xf1.rotation = xf.rotation;
			xf1.translation = xf.translation + m_d;
			
			m_world.DrawShape(xf1, m_shape, b3Color_red);
		}
	}

	void Step()
	{
		scalar dt = g_testSettings->inv_hertz;
		
		b3Quat q = b3QuatRotationY(0.05f * B3_PI * dt);

		m_d = b3Mul(q, m_d);
		
		CastConvex();

		Test::Step();
	}

	static Test* Create()
	{
		return new ConvexCast();
	}

	b3GridMesh<5, 5> m_grid;

	b3Shape* m_shape;
	b3Vec3 m_d;
};

#endif