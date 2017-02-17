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

#include <bounce/common/draw.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>

#include <bounce/dynamics/contacts/convex_contact.h>
#include <bounce/dynamics/contacts/mesh_contact.h>

#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>

#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>

#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/mesh.h>

const b3Color b3Color_black(0.0f, 0.0f, 0.0f);
const b3Color b3Color_white(1.0f, 1.0f, 1.0f);
const b3Color b3Color_red(1.0f, 0.0f, 0.0f);
const b3Color b3Color_green(0.0f, 1.0f, 0.0f);
const b3Color b3Color_blue(0.0f, 0.0f, 1.0f);
const b3Color b3Color_yellow(1.0f, 1.0f, 0.0f);
const b3Color b3Color_pink(1.0f, 0.0f, 1.0f);

void b3World::DebugDraw() const
{
	B3_ASSERT(m_debugDraw);

	u32 flags = m_debugDraw->m_flags;

	if (flags & b3Draw::e_centerOfMassesFlag)
	{
		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			b3Transform xf = b->m_xf;
			xf.position = b->m_sweep.worldCenter;
			m_debugDraw->DrawTransform(xf);
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
				m_debugDraw->DrawAABB(aabb, b3Color_pink);
			}
		}
	}

	if (flags & b3Draw::e_jointsFlag)
	{
		for (b3Joint* j = m_jointMan.m_jointList.m_head; j; j = j->m_next)
		{
			DrawJoint(j);
		}
	}

	for (b3Contact* c = m_contactMan.m_contactList.m_head; c; c = c->m_next)
	{
		const b3Shape* shapeA = c->GetShapeA();
		const b3Shape* shapeB = c->GetShapeB();

		u32 manifoldCount = c->m_manifoldCount;
		const b3Manifold* manifolds = c->m_manifolds;

		for (u32 i = 0; i < manifoldCount; ++i)
		{
			const b3Manifold* m = manifolds + i;

			b3WorldManifold wm;
			wm.Initialize(m, shapeA->GetBody()->GetTransform(), shapeA->m_radius, shapeB->GetBody()->GetTransform(), shapeB->m_radius);

			if (wm.pointCount > 0)
			{
				b3Vec3 n = wm.normal;
				b3Vec3 t1 = wm.tangent1;
				b3Vec3 t2 = wm.tangent2;
				b3Vec3 p = wm.center;
				float32 Pt1 = m->tangentImpulse.x;
				float32 Pt2 = m->tangentImpulse.y;
				
				if (flags & b3Draw::e_contactPointsFlag)
				{
					m_debugDraw->DrawPoint(p, 4.0f, b3Color_yellow);
				}

				if (flags & b3Draw::e_contactNormalsFlag)
				{
					m_debugDraw->DrawSegment(p, p + n, b3Color_yellow);
				}

				if (flags & b3Draw::e_contactTangentsFlag)
				{
					m_debugDraw->DrawSegment(p, p + t1, b3Color_yellow);
					m_debugDraw->DrawSegment(p, p + t2, b3Color_yellow);
				}
			}

			for (u32 j = 0; j < wm.pointCount; ++j)
			{
				const b3ManifoldPoint* mp = m->points + j;
				const b3WorldManifoldPoint* wmp = wm.points + j;

				b3Vec3 n = wmp->normal;
				b3Vec3 t1 = wm.tangent1;
				b3Vec3 t2 = wm.tangent2;
				b3Vec3 p = wmp->point;
				float32 Pn = mp->normalImpulse;
				float32 Pt1 = mp->tangentImpulse.x;
				float32 Pt2 = mp->tangentImpulse.y;

				if (flags & b3Draw::e_contactPointsFlag)
				{
					m_debugDraw->DrawPoint(p, 4.0f, mp->persisting ? b3Color_green : b3Color_red);
				}
				
				if (flags & b3Draw::e_contactNormalsFlag)
				{
					m_debugDraw->DrawSegment(p, p + n, b3Color_white);
				}
				
				if (flags & b3Draw::e_contactTangentsFlag)
				{
					m_debugDraw->DrawSegment(p, p + t1, b3Color_yellow);
					m_debugDraw->DrawSegment(p, p + t2, b3Color_yellow);
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
		m_debugDraw->DrawPoint(p, 4.0f, wireColor);
		break;
	}
	case e_capsuleShape:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		b3Vec3 p1 = xf * capsule->m_centers[0];
		b3Vec3 p2 = xf * capsule->m_centers[1];
		m_debugDraw->DrawPoint(p1, 4.0f, wireColor);
		m_debugDraw->DrawPoint(p2, 4.0f, wireColor);
		m_debugDraw->DrawSegment(p1, p2, wireColor);
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

			m_debugDraw->DrawSegment(p1, p2, wireColor);
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

			m_debugDraw->DrawTriangle(p1, p2, p3, wireColor);
		}
		break;
	}
	default:
	{
		break;
	}
	};
}

void b3World::DrawJoint(const b3Joint* joint) const
{
	b3JointType type = joint->GetType();

	switch (type)
	{
	case e_mouseJoint:
	{
		b3MouseJoint* o = (b3MouseJoint*)joint;
		o->Draw(m_debugDraw);
		break;
	}
	case e_springJoint:
	{
		b3SpringJoint* o = (b3SpringJoint*)joint;
		o->Draw(m_debugDraw);
		break;
	}
	case e_revoluteJoint:
	{
		b3RevoluteJoint* o = (b3RevoluteJoint*)joint;
		o->Draw(m_debugDraw);
		break;
	}
	case e_sphereJoint:
	{
		b3SphereJoint* o = (b3SphereJoint*)joint;
		o->Draw(m_debugDraw);
		break;
	}
	case e_coneJoint:
	{
		b3ConeJoint* o = (b3ConeJoint*)joint;
		o->Draw(m_debugDraw);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

void b3World::DrawContact(const b3Contact* c) const
{

}
