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

#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/mesh.h>

b3Shape::b3Shape() 
{
	m_density = 0.0f;
	m_friction = 0.0f;
	m_restitution = 0.0f;
	
	m_isSensor = false;
	m_userData = nullptr;

	m_body = nullptr;
}

void b3Shape::SetSensor(bool flag)
{
	if (flag != m_isSensor)
	{
		if (m_body)
		{
			m_body->SetAwake(true);
		}
		m_isSensor = flag;
	}
}

void b3Shape::DestroyContacts()
{
	b3World* world = m_body->GetWorld();
	b3ContactEdge* ce = m_contactEdges.m_head;
	while (ce)
	{
		b3ContactEdge* tmp = ce;
		ce = ce->m_next;
		world->m_contactMan.Destroy(tmp->contact);
	}
}

void b3Shape::Dump(u32 bodyIndex) const
{
	switch (m_type)
	{
	case e_sphereShape:
	{
		b3SphereShape* sphere = (b3SphereShape*) this;
		b3Log("		b3SphereShape shape;\n");
		b3Log("		shape.m_center.Set(%f, %f, %f);\n", sphere->m_center.x, sphere->m_center.y, sphere->m_center.z);
		b3Log("		shape.m_radius = %f;\n", sphere->m_radius);
		break;
	}
	case e_capsuleShape:
	{
		b3CapsuleShape* capsule = (b3CapsuleShape*) this;
		b3Log("		b3CapsuleShape shape;\n");
		b3Log("		shape.m_centers[0].Set(%f, %f, %f);\n", capsule->m_centers[0].x, capsule->m_centers[0].y, capsule->m_centers[0].z);
		b3Log("		shape.m_centers[1].Set(%f, %f, %f);\n", capsule->m_centers[1].x, capsule->m_centers[1].y, capsule->m_centers[1].z);
		b3Log("		shape.m_radius = %f;\n", capsule->m_radius);
		break;
	}
	case e_hullShape:
	{
		b3HullShape* hs = (b3HullShape*) this;
		const b3Hull* h = hs->m_hull;
		
		b3Log("		u8* marker = (u8*) b3Alloc(%d);\n", h->GetSize());
		b3Log("		\n");
		b3Log("		b3Hull* h = (b3Hull*)marker;\n");
		b3Log("		marker += 1 * sizeof(b3Hull);\n");
		b3Log("		h->vertices = (b3Vec3*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Vec3);\n", h->vertexCount);
		b3Log("		h->edges = (b3HalfEdge*)marker;\n");
		b3Log("		marker += %d * sizeof(b3HalfEdge);\n", h->edgeCount);
		b3Log("		h->faces = (b3Face*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Face);\n", h->faceCount);
		b3Log("		h->planes = (b3Plane*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Plane);\n", h->faceCount);
		b3Log("		\n");
		b3Log("		h->centroid.Set(%f, %f, %f);\n", h->centroid.x, h->centroid.y, h->centroid.z);
		b3Log("		\n");
		b3Log("		h->vertexCount = %d;\n", h->vertexCount);
		for (u32 i = 0; i < h->vertexCount; ++i)
		{
			const b3Vec3* v = h->vertices + i;
			b3Log("		h->vertices[%d].Set(%f, %f, %f);\n", i, v->x, v->y, v->z);
		}
		b3Log("		\n");
		b3Log("		h->edgeCount = %d;\n", h->edgeCount);
		for (u32 i = 0; i < h->edgeCount; ++i)
		{
			const b3HalfEdge* e = h->edges + i;
			b3Log("		h->edges[%d].origin = %d;\n", i, e->origin);
			b3Log("		h->edges[%d].twin = %d;\n", i, e->twin);
			b3Log("		h->edges[%d].face = %d;\n", i, e->face);
			b3Log("		h->edges[%d].next = %d;\n", i, e->next);
		}
		b3Log("		\n");
		b3Log("		h->faceCount = %d;\n", h->faceCount);
		for (u32 i = 0; i < h->faceCount; ++i)
		{
			const b3Face* f = h->faces + i;
			b3Log("		h->faces[%d].edge = %d;\n", i, f->edge);
		}
		b3Log("		\n");
		for (u32 i = 0; i < h->faceCount; ++i)
		{
			const b3Plane* p = h->planes + i;
			b3Log("		h->planes[%d].normal.Set(%f, %f, %f);\n", i, p->normal.x, p->normal.y, p->normal.z);
			b3Log("		h->planes[%d].offset = %f;\n", i, p->offset);
		}
		b3Log("		\n");
		b3Log("		h->Validate();\n");
		b3Log("		\n");
		b3Log("		b3HullShape shape;\n");
		b3Log("		shape.m_hull = h;\n");
		b3Log("		shape.m_radius = %f;\n", m_radius);
		break;
	}
	case e_meshShape:
	{
		b3MeshShape* ms = (b3MeshShape*) this;
		const b3Mesh* m = ms->m_mesh;
		
		b3Log("		u8* marker = (u8*) b3Alloc(%d);\n", m->GetSize());
		b3Log("		\n");
		b3Log("		b3Mesh* m = (b3Hull*)marker;\n");
		b3Log("		marker += 1 * sizeof(b3Mesh);\n");
		b3Log("		m->vertices = (b3Vec3*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Vec3);\n", m->vertexCount);
		b3Log("		m->triangles = (b3Triangle*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Triangle);\n", m->triangleCount);
		b3Log("		m->planes = (b3Plane*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Plane);\n", 2 * m->triangleCount);
		b3Log("		\n");
		for (u32 i = 0; i < m->vertexCount; ++i)
		{
			const b3Vec3* v = m->vertices + i;
			b3Log("		m->vertices[%d].Set(%f, %f, %f);\n", i, v->x, v->y, v->z);
		}
		b3Log("		\n");
		for (u32 i = 0; i < m->triangleCount; ++i)
		{
			const b3Triangle* t = m->triangles + i;
			b3Log("		m->triangles[%d].v1 = %d;\n", i, t->v1);
			b3Log("		m->triangles[%d].v2 = %d;\n", i, t->v2);
			b3Log("		m->triangles[%d].v3 = %d;\n", i, t->v3);
		}
		b3Log("		\n");
		b3Log("		\n");
		b3Log("		m->BuildTree();\n");		
		b3Log("		\n");
		b3Log("		b3MeshShape shape;\n");
		b3Log("		shape.m_mesh = m;\n");
		b3Log("		shape.m_radius = %f;\n", m_radius);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	};

	b3Log("		\n");
	b3Log("		b3ShapeDef sd;\n");
	b3Log("		sd.shape = &shape;\n");
	b3Log("		sd.density = %f;\n", m_density);
	b3Log("		sd.restitution = %f;\n", m_restitution);
	b3Log("		sd.friction = %f;\n", m_friction);
	b3Log("		sd.sensor = %d;\n", m_isSensor);
	b3Log("		\n");
	b3Log("		bodies[%d]->CreateShape(sd);\n", bodyIndex);
}

b3Shape* b3Shape::Create(const b3ShapeDef& def)
{
	b3Shape* shape = NULL;
	switch (def.shape->GetType())
	{
	case e_sphereShape:
	{
		// Grab pointer to the specific memory.
		b3SphereShape* sphere1 = (b3SphereShape*)def.shape;
		void* mem = b3Alloc(sizeof(b3SphereShape));
		b3SphereShape* sphere2 = new (mem)b3SphereShape();
		// Clone the polyhedra.
		sphere2->Swap(*sphere1);
		shape = sphere2;
		break;
	}
	case e_capsuleShape:
	{
		// Grab pointer to the specific memory.
		b3CapsuleShape* caps1 = (b3CapsuleShape*)def.shape;
		void* block = b3Alloc(sizeof(b3CapsuleShape));
		b3CapsuleShape* caps2 = new (block)b3CapsuleShape();
		caps2->Swap(*caps1);
		shape = caps2;
		break;
	}
	case e_hullShape:
	{
		// Grab pointer to the specific memory.
		b3HullShape* hull1 = (b3HullShape*)def.shape;
		void* block = b3Alloc(sizeof(b3HullShape));
		b3HullShape* hull2 = new (block)b3HullShape();
		hull2->Swap(*hull1);
		shape = hull2;
		break;
	}
	case e_meshShape:
	{
		// Grab pointer to the specific memory.
		b3MeshShape* mesh1 = (b3MeshShape*)def.shape;
		void* block = b3Alloc(sizeof(b3MeshShape));
		b3MeshShape* mesh2 = new (block) b3MeshShape();
		// Clone the mesh.
		mesh2->Swap(*mesh1);
		shape = mesh2;
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}

	return shape;
}

void b3Shape::Destroy(b3Shape* shape)
{
	// Free the shape from the memory.
	switch (shape->GetType())
	{
	case e_sphereShape:
	{
		b3SphereShape* sphere = (b3SphereShape*)shape;
		sphere->~b3SphereShape();
		b3Free(shape);
		break;
	}
	case e_capsuleShape:
	{
		b3CapsuleShape* caps = (b3CapsuleShape*)shape;
		caps->~b3CapsuleShape();
		b3Free(shape);
		break;
	}
	case e_hullShape:
	{
		b3HullShape* hull = (b3HullShape*)shape;
		hull->~b3HullShape();
		b3Free(shape);
		break;
	}
	case e_meshShape:
	{
		b3MeshShape* mesh = (b3MeshShape*)shape;
		mesh->~b3MeshShape();
		b3Free(shape);
		break;
	}
	default:
	{
		B3_ASSERT(false);
	}
	}
}
