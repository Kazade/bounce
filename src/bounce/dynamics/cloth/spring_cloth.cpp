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

#include <bounce/dynamics/cloth/spring_cloth.h>
#include <bounce/dynamics/cloth/spring_solver.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/common/memory/stack_allocator.h>

#define B3_FORCE_THRESHOLD (0.1f)

b3SpringCloth::b3SpringCloth()
{
	m_allocator = nullptr;

	m_mesh = nullptr;

	m_gravity.SetZero();

	m_x = nullptr;
	m_v = nullptr;
	m_f = nullptr;
	m_y = nullptr;
	m_inv_m = nullptr;
	m_types = nullptr;
	m_contacts = nullptr;
	m_massCount = 0;

	m_springs = nullptr;
	m_springCount = 0;

	m_r = 0.0f;

	m_shapeCount = 0;

	m_step.iterations = 0;
}

b3SpringCloth::~b3SpringCloth()
{
	b3Free(m_x);
	b3Free(m_v);
	b3Free(m_f);
	b3Free(m_inv_m);
	b3Free(m_y);
	b3Free(m_types);
	b3Free(m_contacts);
	b3Free(m_springs);
}

void b3SpringCloth::Initialize(const b3SpringClothDef& def)
{
	B3_ASSERT(def.allocator);
	B3_ASSERT(def.mesh);

	m_allocator = def.allocator;
	m_mesh = def.mesh;
	m_gravity = def.gravity;

	m_r = def.r;

	const b3Mesh* m = m_mesh;

	m_massCount = m->vertexCount;
	m_x = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_v = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_f = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_inv_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_y = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_types = (b3MassType*)b3Alloc(m_massCount * sizeof(b3MassType));
	m_contacts = (b3MassContact*)b3Alloc(m_massCount * sizeof(b3MassContact));

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		m_contacts[i].Fn = 0.0f;
		m_contacts[i].Ft1 = 0.0f;
		m_contacts[i].Ft2 = 0.0f;
		m_contacts[i].lockOnSurface = false;
		m_contacts[i].slideOnSurface = false;

		m_x[i] = m->vertices[i];
		m_v[i].SetZero();
		m_f[i].SetZero();
		m_inv_m[i] = 0.0f;
		m_y[i].SetZero();
		m_types[i] = e_staticMass;
	}

	// Initialize mass
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Vec3 p1 = m->vertices[t->v1];
		b3Vec3 p2 = m->vertices[t->v2];
		b3Vec3 p3 = m->vertices[t->v3];

		float32 area = b3Area(p1, p2, p3);
		float32 mass = def.density * area;

		const float32 inv3 = 1.0f / 3.0f;

		m_inv_m[t->v1] += inv3 * mass;
		m_inv_m[t->v2] += inv3 * mass;
		m_inv_m[t->v3] += inv3 * mass;
	}

	// Invert
	for (u32 i = 0; i < m_massCount; ++i)
	{
		if (m_inv_m[i] > 0.0f)
		{
			m_inv_m[i] = 1.0f / m_inv_m[i];
			m_types[i] = e_dynamicMass;
		}
	}

	// Initialize springs
	m_springs = (b3Spring*)b3Alloc(3 * m->triangleCount * sizeof(b3Spring));

	// Streching
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		u32 is[3] = { t->v1, t->v2, t->v3 };
		for (u32 j = 0; j < 3; ++j)
		{
			u32 k = j + 1 < 3 ? j + 1 : 0;

			u32 v1 = is[j];
			u32 v2 = is[k];

			b3Vec3 p1 = m->vertices[v1];
			b3Vec3 p2 = m->vertices[v2];

			// Skip duplicated spring
			bool found = false;
			for (u32 s = 0; s < m_springCount; ++s)
			{
				if ((m_springs[s].i1 == v1 && m_springs[s].i2 == v2) || (m_springs[s].i1 == v2 && m_springs[s].i2 == v1))
				{
					found = true;
					break;
				}
			}

			if (found == false)
			{
				b3Spring* S = m_springs + m_springCount;
				S->i1 = v1;
				S->i2 = v2;
				S->L0 = b3Distance(p1, p2);
				S->ks = def.ks;
				S->kd = def.kd;
				++m_springCount;
			}
		}
	}
}

void b3SpringCloth::AddShape(b3Shape* shape)
{
	B3_ASSERT(m_shapeCount < B3_CLOTH_SHAPE_CAPACITY);

	if (m_shapeCount == B3_CLOTH_SHAPE_CAPACITY)
	{
		return;
	}

	m_shapes[m_shapeCount++] = shape;
}

static B3_FORCE_INLINE void b3MakeTangents(b3Vec3& t1, b3Vec3& t2, const b3Vec3& dv, const b3Vec3& n)
{
	t1 = dv - b3Dot(dv, n) * n;
	if (b3Dot(t1, t1) > B3_EPSILON * B3_EPSILON)
	{
		t1.Normalize();
		t2 = b3Cross(t1, n);
	}
	else
	{
		t1 = b3Perp(n);
		t2 = b3Cross(t1, n);
	}
}

void b3SpringCloth::UpdateContacts()
{
	for (u32 i = 0; i < m_massCount; ++i)
	{
		b3MassContact* c = m_contacts + i;

		bool wasLocked = c->lockOnSurface;
		bool wasSliding = c->slideOnSurface;

		b3Sphere s1;
		s1.vertex = m_x[i];
		s1.radius = m_r;

		// Solve the deepest penetration
		float32 bestSeparation = 0.0f;
		b3Vec3 bestNormal(0.0f, 0.0f, 0.0f);
		u32 bestIndex = ~0;

		for (u32 j = 0; j < m_shapeCount; ++j)
		{
			b3Shape* shape = m_shapes[j];

			b3Transform xf2;
			xf2.SetIdentity();

			b3TestSphereOutput output;
			if (shape->TestSphere(&output, s1, xf2) == false)
			{
				continue;
			}

			if (output.separation < bestSeparation)
			{
				bestSeparation = output.separation;
				bestNormal = output.normal;
				bestIndex = j;
			}
		}

		if (bestIndex == ~0)
		{
			c->Fn = 0.0f;
			c->Ft1 = 0.0f;
			c->Ft2 = 0.0f;
			c->lockOnSurface = false;
			c->slideOnSurface = false;
			continue;
		}

		B3_ASSERT(bestSeparation <= 0.0f);

		const b3Shape* shape = m_shapes[bestIndex];
		float32 s = bestSeparation;
		
		// Ensure the normal points to the shape
		b3Vec3 n = -bestNormal;

		// Apply position correction
		b3Vec3 dx = s * n;
		
		m_y[i] += dx;

		// Update contact state
		if (wasLocked)
		{
			// Was the contact force attractive?
			if (c->Fn > -B3_FORCE_THRESHOLD)
			{
				// Terminate the contact.
				c->lockOnSurface = false;
				continue;
			}

			// Since the contact force was repulsive 
			// maintain the acceleration constraint.
			c->n = n;
			c->j = bestIndex;
			c->lockOnSurface = true;
		}
		else
		{
			// The contact has began.
			c->n = n;
			c->Fn = 0.0f;
			c->Ft1 = 0.0f;
			c->Ft2 = 0.0f;
			c->j = bestIndex;
			c->lockOnSurface = true;

			// Relative velocity
			b3Vec3 dv = m_v[i];
			
			b3MakeTangents(c->t1, c->t2, dv, n);
			c->slideOnSurface = false;

			continue;
		}
	}
}

void b3SpringCloth::Step(float32 dt)
{
	if (dt == 0.0f)
	{
		return;
	}

	// Update contacts
	UpdateContacts();

	// Apply gravity
	for (u32 i = 0; i < m_massCount; ++i)
	{
		if (m_types[i] == e_dynamicMass)
		{
			m_f[i] += m_gravity;
		}
	}

	// Solve springs, constraints, and integrate
	b3SpringSolverDef solverDef;
	solverDef.cloth = this;
	solverDef.dt = dt;

	b3SpringSolver solver(solverDef);

	// Constraint forces that are required to satisfy the constraints
	b3Vec3* forces = (b3Vec3*)m_allocator->Allocate(m_massCount * sizeof(b3Vec3));

	solver.Solve(forces);

	// Store constraint forces for physics logic
	for (u32 i = 0; i < m_massCount; ++i)
	{
		b3Vec3 force = forces[i];

		b3MassContact* contact = m_contacts + i;

		// Signed normal force magnitude
		contact->Fn = b3Dot(force, contact->n);

		// Signed tangent forces magnitude
		contact->Ft1 = b3Dot(force, contact->t1);
		contact->Ft2 = b3Dot(force, contact->t2);
	}

	m_allocator->Free(forces);

	// Clear position correction
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_y[i].SetZero();
	}

	// Clear forces
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_f[i].SetZero();
	}
}

void b3SpringCloth::Apply() const
{
	for (u32 i = 0; i < m_massCount; ++i)
	{
		m_mesh->vertices[i] = m_x[i];
	}
}

void b3SpringCloth::Draw(b3Draw* draw) const
{
	const b3Mesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		if (m_contacts[i].lockOnSurface)
		{
			if (m_contacts[i].Fn > -B3_FORCE_THRESHOLD)
			{
				draw->DrawPoint(m_x[i], 6.0f, b3Color_yellow);
			}
			else
			{
				draw->DrawPoint(m_x[i], 6.0f, b3Color_red);
			}
		}
		else
		{
			draw->DrawPoint(m_x[i], 6.0f, b3Color_green);
		}
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Vec3 v1 = m_x[t->v1];
		b3Vec3 v2 = m_x[t->v2];
		b3Vec3 v3 = m_x[t->v3];

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();

		b3Vec3 n2 = -n1;

		draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);
		draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}