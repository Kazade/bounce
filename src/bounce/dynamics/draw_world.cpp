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

#include <bounce/bounce.h>

const b3Color b3Color_black(0.0f, 0.0f, 0.0f);
const b3Color b3Color_white(1.0f, 1.0f, 1.0f);
const b3Color b3Color_red(1.0f, 0.0f, 0.0f);
const b3Color b3Color_green(0.0f, 1.0f, 0.0f);
const b3Color b3Color_blue(0.0f, 0.0f, 1.0f);
const b3Color b3Color_yellow(1.0f, 1.0f, 0.0f);
const b3Color b3Color_pink(1.0f, 0.0f, 1.0f);

b3Draw* b3Draw_draw(nullptr);

void b3World::Draw() const
{
	B3_ASSERT(b3Draw_draw);

	u32 flags = b3Draw_draw->m_flags;

	if (flags & b3Draw::e_centerOfMassesFlag)
	{
		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			b3Transform xf = b->m_xf;
			xf.position = b->m_sweep.worldCenter;
			b3Draw_draw->DrawTransform(xf);
		}
	}

	if (flags & b3Draw::e_shapesFlag)
	{
		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			const b3Transform& xf = b->GetTransform();
			for (b3Shape* s = b->m_shapeList.m_head; s; s = s->m_next)
			{
				DrawShape(xf, s);
			}
		}
	}

	if (flags & b3Draw::e_aabbsFlag)
	{
		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			for (b3Shape* s = b->m_shapeList.m_head; s; s = s->m_next)
			{
				const b3AABB3& aabb = m_contactMan.m_broadPhase.GetAABB(s->m_broadPhaseID);
				b3Draw_draw->DrawAABB(aabb, b3Color_pink);
			}
		}
	}

	if (flags & b3Draw::e_jointsFlag)
	{
		for (b3Joint* j = m_jointMan.m_jointList.m_head; j; j = j->m_next)
		{
			j->Draw();
		}
	}

	for (b3Contact* c = m_contactMan.m_contactList.m_head; c; c = c->m_next)
	{
		u32 manifoldCount = c->m_manifoldCount;
		const b3Manifold* manifolds = c->m_manifolds;

		for (u32 i = 0; i < manifoldCount; ++i)
		{
			const b3Manifold* m = manifolds + i;
			b3WorldManifold wm;
			c->GetWorldManifold(&wm, i);

			b3Vec3 t1 = wm.tangent1;
			b3Vec3 t2 = wm.tangent2;

			b3Vec3 points[B3_MAX_MANIFOLD_POINTS];
			for (u32 j = 0; j < m->pointCount; ++j)
			{
				const b3ManifoldPoint* mp = m->points + j;
				const b3WorldManifoldPoint* wmp = wm.points + j;

				b3Vec3 n = wmp->normal;
				b3Vec3 p = wmp->point;

				points[j] = p;

				if (flags & b3Draw::e_contactPointsFlag)
				{
					b3Draw_draw->DrawPoint(p, 4.0f, mp->persisting ? b3Color_green : b3Color_red);
				}

				if (flags & b3Draw::e_contactNormalsFlag)
				{
					b3Draw_draw->DrawSegment(p, p + n, b3Color_white);
				}
			}

			if (m->pointCount > 0)
			{
				b3Vec3 p = wm.center;
				b3Vec3 n = wm.normal;
				t1 = wm.tangent1;
				t2 = wm.tangent2;

				if (flags & b3Draw::e_contactNormalsFlag)
				{
					b3Draw_draw->DrawSegment(p, p + n, b3Color_yellow);
				}

				if (flags & b3Draw::e_contactTangentsFlag)
				{
					b3Draw_draw->DrawSegment(p, p + t1, b3Color_yellow);
					b3Draw_draw->DrawSegment(p, p + t2, b3Color_yellow);
				}

				if (m->pointCount > 2)
				{
					if (flags & b3Draw::e_contactPolygonsFlag)
					{
						b3Draw_draw->DrawSolidPolygon(wm.normal, points, m->pointCount, b3Color_pink);
					}
				}
			}
		}
	}
}

void b3World::DrawShape(const b3Transform& xf, const b3Shape* shape) const
{
	b3Color wireColor(0.0f, 0.0f, 0.0f);
	switch (shape->GetType())
	{
	case e_sphereShape:
	{
		const b3SphereShape* sphere = (b3SphereShape*)shape;
		b3Vec3 p = xf * sphere->m_center;
		b3Draw_draw->DrawPoint(p, 4.0f, wireColor);
		break;
	}
	case e_capsuleShape:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		b3Vec3 p1 = xf * capsule->m_centers[0];
		b3Vec3 p2 = xf * capsule->m_centers[1];
		b3Draw_draw->DrawPoint(p1, 4.0f, wireColor);
		b3Draw_draw->DrawPoint(p2, 4.0f, wireColor);
		b3Draw_draw->DrawSegment(p1, p2, wireColor);
		break;
	}
	case e_hullShape:
	{
		const b3HullShape* hs = (b3HullShape*)shape;
		const b3Hull* hull = hs->m_hull;
		for (u32 i = 0; i < hull->edgeCount; i += 2)
		{
			const b3HalfEdge* edge = hull->GetEdge(i);
			const b3HalfEdge* twin = hull->GetEdge(i + 1);

			b3Vec3 p1 = xf * hull->vertices[edge->origin];
			b3Vec3 p2 = xf * hull->vertices[twin->origin];

			b3Draw_draw->DrawSegment(p1, p2, wireColor);
		}
		break;
	}
	case e_meshShape:
	{
		const b3MeshShape* ms = (b3MeshShape*)shape;
		const b3Mesh* mesh = ms->m_mesh;
		for (u32 i = 0; i < mesh->triangleCount; ++i)
		{
			const b3Triangle* t = mesh->triangles + i;

			b3Vec3 p1 = xf * mesh->vertices[t->v1];
			b3Vec3 p2 = xf * mesh->vertices[t->v2];
			b3Vec3 p3 = xf * mesh->vertices[t->v3];

			b3Draw_draw->DrawTriangle(p1, p2, p3, wireColor);
		}
		break;
	}
	default:
	{
		break;
	}
	};
}