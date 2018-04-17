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
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/sparse_mat33.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/draw.h>

#define B3_FORCE_THRESHOLD (0.1f)

#define B3_CLOTH_BENDING 0

#define B3_CLOTH_FRICTION 0

b3SpringCloth::b3SpringCloth()
{
	m_allocator = nullptr;

	m_mesh = nullptr;

	m_gravity.SetZero();

	m_x = nullptr;
	m_v = nullptr;
	m_f = nullptr;
	m_y = nullptr;
	m_m = nullptr;
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
	b3Free(m_m);
	b3Free(m_y);
	b3Free(m_types);
	b3Free(m_contacts);
	b3Free(m_springs);
}

static B3_FORCE_INLINE u32 b3NextIndex(u32 i)
{
	return i + 1 < 3 ? i + 1 : 0;
}

struct b3UniqueEdge
{
	u32 v1, v2;
};

struct b3SharedEdge
{
	u32 v1, v2;
	u32 nsv1, nsv2;
};

static void b3FindEdges(b3UniqueEdge* uniqueEdges, u32& uniqueCount, b3SharedEdge* sharedEdges, u32& sharedCount, const b3Mesh* m)
{
	uniqueCount = 0;
	sharedCount = 0;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t1 = m->triangles + i;
		u32 i1s[3] = { t1->v1, t1->v2, t1->v3 };

		for (u32 j1 = 0; j1 < 3; ++j1)
		{
			u32 t1v1 = i1s[j1];
			u32 t1v2 = i1s[b3NextIndex(j1)];

			bool unique = true;

			for (u32 j = 0; j < uniqueCount; ++j)
			{
				b3UniqueEdge* ue = uniqueEdges + j;

				if (ue->v1 == t1v1 && ue->v2 == t1v2)
				{
					unique = false;
					break;
				}
				
				if (ue->v2 == t1v1 && ue->v1 == t1v2)
				{
					unique = false;
					break;
				}
			}

			if (unique)
			{
				b3UniqueEdge ue;
				ue.v1 = t1v1;
				ue.v2 = t1v2;
				uniqueEdges[uniqueCount++] = ue;
			}
		}
	}
}

void b3SpringCloth::Initialize(const b3SpringClothDef& def)
{
	B3_ASSERT(def.allocator);
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_allocator = def.allocator;

	m_mesh = def.mesh;
	m_r = def.r;

	m_gravity = def.gravity;

	const b3Mesh* m = m_mesh;

	m_massCount = m->vertexCount;
	m_x = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_v = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_f = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_inv_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_y = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_types = (b3MassType*)b3Alloc(m_massCount * sizeof(b3MassType));
	m_contacts = (b3MassContact*)b3Alloc(m_massCount * sizeof(b3MassContact));

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		m_contacts[i].Fn = 0.0f;
		m_contacts[i].Ft1 = 0.0f;
		m_contacts[i].Ft2 = 0.0f;
		m_contacts[i].lockN = false;
		m_contacts[i].lockT1 = false;
		m_contacts[i].lockT2 = false;

		m_x[i] = m->vertices[i];
		m_v[i].SetZero();
		m_f[i].SetZero();
		m_m[i] = 0.0f;
		m_inv_m[i] = 0.0f;
		m_y[i].SetZero();
		m_types[i] = b3MassType::e_staticMass;
	}

	// Initialize mass
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3Triangle* t = m->triangles + i;

		b3Vec3 p1 = m->vertices[t->v1];
		b3Vec3 p2 = m->vertices[t->v2];
		b3Vec3 p3 = m->vertices[t->v3];

		float32 area = b3Area(p1, p2, p3);

		B3_ASSERT(area > B3_EPSILON);

		float32 mass = def.density * area;

		const float32 inv3 = 1.0f / 3.0f;

		m_m[t->v1] += inv3 * mass;
		m_m[t->v2] += inv3 * mass;
		m_m[t->v3] += inv3 * mass;
	}

	// Invert
	for (u32 i = 0; i < m_massCount; ++i)
	{
		B3_ASSERT(m_m[i] > 0.0f);
		m_inv_m[i] = 1.0f / m_m[i];
		m_types[i] = b3MassType::e_dynamicMass;
	}

	// Initialize springs
	u32 edgeCount = 3 * m->triangleCount;

	b3SharedEdge* sharedEdges = (b3SharedEdge*)m_allocator->Allocate(edgeCount * sizeof(b3SharedEdge));
	u32 sharedCount = 0;

	b3UniqueEdge* uniqueEdges = (b3UniqueEdge*)m_allocator->Allocate(edgeCount * sizeof(b3UniqueEdge));
	u32 uniqueCount = 0;

	b3FindEdges(uniqueEdges, uniqueCount, sharedEdges, sharedCount, m);

	u32 springCapacity = uniqueCount + sharedCount;
	m_springs = (b3Spring*)b3Alloc(springCapacity * sizeof(b3Spring));

	// Streching
	for (u32 i = 0; i < uniqueCount; ++i)
	{
		b3UniqueEdge* e = uniqueEdges + i;

		b3Vec3 p1 = m->vertices[e->v1];
		b3Vec3 p2 = m->vertices[e->v2];

		b3Spring* S = m_springs + m_springCount;
		S->type = e_strechSpring;
		S->i1 = e->v1;
		S->i2 = e->v2;
		S->L0 = b3Distance(p1, p2);
		S->ks = def.ks;
		S->kd = def.kd;
		++m_springCount;
	}

#if B3_CLOTH_BENDING == 1
	// Bending
	for (u32 i = 0; i < sharedCount; ++i)
	{
		b3SharedEdge* e = sharedEdges + i;

		b3Vec3 p1 = m->vertices[e->nsv1];
		b3Vec3 p2 = m->vertices[e->nsv2];

		b3Spring* S = m_springs + m_springCount;
		S->type = e_bendSpring;
		S->i1 = e->nsv1;
		S->i2 = e->nsv2;
		S->L0 = b3Distance(p1, p2);
		S->ks = def.kb;
		S->kd = def.kd;
		++m_springCount;
	}

#endif // #if B3_CLOTH_BENDING

	m_allocator->Free(uniqueEdges);
	m_allocator->Free(sharedEdges);

	B3_ASSERT(m_springCount <= springCapacity);
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

void b3SpringCloth::GetTension(b3Array<b3Vec3>& T) const
{
	B3_ASSERT(T.Count() == 0);

	T.Resize(m_massCount);

	for (u32 i = 0; i < T.Count(); ++i)
	{
		T[i].SetZero();
	}

	// T = F - mg
	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* S = m_springs + i;

		b3SpringType type = S->type;
		u32 i1 = S->i1;
		u32 i2 = S->i2;
		float32 L0 = S->L0;
		float32 ks = S->ks;
		float32 kd = S->kd;

		b3Vec3 x1 = m_x[i1];
		b3Vec3 v1 = m_v[i1];

		b3Vec3 x2 = m_x[i2];
		b3Vec3 v2 = m_v[i2];

		// Strech
		b3Vec3 dx = x1 - x2;
		float32 L = b3Length(dx);

		float32 s = 1.0f;
		if (L > 0.0f)
		{
			s -= L0 / L;
		}

		b3Vec3 sf1 = -ks * s * dx;
		b3Vec3 sf2 = -sf1;

		T[i1] += sf1;
		T[i2] += sf2;

		// Damping
		b3Vec3 dv = v1 - v2;

		b3Vec3 df1 = -kd * dv;
		b3Vec3 df2 = -df1;

		T[i1] += df1;
		T[i2] += df2;
	}

	for (u32 i = 0; i < T.Count(); ++i)
	{
		if (m_types[i] == b3MassType::e_staticMass)
		{
			T[i].SetZero();
		}
	}
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
		// Static masses can't participate in collisions.
		if (m_types[i] == b3MassType::e_staticMass)
		{
			continue;
		}

		b3MassContact* c = m_contacts + i;

		bool wasLockedN = c->lockN;

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
			c->lockN = false;
			c->lockT1 = false;
			c->lockT2 = false;
			continue;
		}

		B3_ASSERT(bestSeparation <= 0.0f);

		b3Shape* shape = m_shapes[bestIndex];
		float32 s = bestSeparation;
		b3Vec3 n = bestNormal;

		// Apply position correction
		m_y[i] -= s * n;

		// Update contact state
		if (wasLockedN)
		{
			// Was the contact force attractive?
			if (c->Fn < B3_FORCE_THRESHOLD)
			{
				// Terminate the contact.
				c->Fn = 0.0f;
				c->Ft1 = 0.0f;
				c->Ft2 = 0.0f;
				c->lockN = false;
				c->lockT1 = false;
				c->lockT2 = false;
				continue;
			}

			// Since the contact force was repulsive 
			// maintain the normal acceleration constraint.
			c->j = bestIndex;
			c->n = n;
		}
		else
		{
			// The contact has began.
			c->j = bestIndex;
			c->n = n;
			c->Fn = 0.0f;
			c->Ft1 = 0.0f;
			c->Ft2 = 0.0f;
			c->lockN = true;
			c->lockT1 = false;
			c->lockT2 = false;
		}

#if B3_CLOTH_FRICTION == 1

		// Apply friction impulses

		// Relative velocity
		b3Vec3 dv = m_v[i];

		b3MakeTangents(c->t1, c->t2, dv, n);

		// Note without a friction force, the tangential acceleration won't be 
		// removed.

		// Coefficients of friction for the solid
		const float32 uk = shape->GetFriction();
		const float32 us = 2.0f * uk;

		float32 dvn = b3Dot(dv, n);
		float32 normalImpulse = -m_inv_m[i] * dvn;

		b3Vec3 ts[2];
		ts[0] = c->t1;
		ts[1] = c->t2;

		bool lockT[2];

		for (u32 k = 0; k < 2; ++k)
		{
			b3Vec3 t = ts[k];

			float32 dvt = b3Dot(dv, t);
			float32 tangentImpulse = -m_inv_m[i] * dvt;

			float32 maxStaticImpulse = us * normalImpulse;
			if (tangentImpulse * tangentImpulse > maxStaticImpulse * maxStaticImpulse)
			{
				lockT[k] = false;

				// Dynamic friction
				float32 maxDynamicImpulse = uk * normalImpulse;
				if (tangentImpulse * tangentImpulse > maxDynamicImpulse * maxDynamicImpulse)
				{
					b3Vec3 P = tangentImpulse * t;

					m_v[i] += m_m[i] * P;
				}
			}
			else
			{
				lockT[k] = true;

				// Static friction
				b3Vec3 P = tangentImpulse * t;

				m_v[i] += m_m[i] * P;
			}
		}

		c->lockT1 = lockT[0];
		c->lockT2 = lockT[1];

#endif // #if B3_CLOTH_FRICTION

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

	// Apply gravity forces
	for (u32 i = 0; i < m_massCount; ++i)
	{
		if (m_types[i] == b3MassType::e_dynamicMass)
		{
			m_f[i] += m_m[i] * m_gravity;
		}
	}

	// Integrate

	b3SpringSolverDef solverDef;
	solverDef.cloth = this;
	solverDef.dt = dt;

	b3SpringSolver solver(solverDef);

	// Extra constraint forces that should have been applied to satisfy the constraints
	// todo Find the applied constraint forces.
	b3DenseVec3 forces(m_massCount);

	solver.Solve(forces);

	m_step.iterations = solver.GetIterations();

	// Store constraint forces for physics logic
	for (u32 i = 0; i < m_massCount; ++i)
	{
		b3Vec3 force = forces[i];

		b3MassContact* c = m_contacts + i;

		// Signed normal force magnitude
		c->Fn = b3Dot(force, c->n);

		// Signed tangent forces magnitude
		c->Ft1 = b3Dot(force, c->t1);
		c->Ft2 = b3Dot(force, c->t2);
	}

	// Clear position alteration
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

void b3SpringCloth::Draw() const
{
	const b3Mesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		if (m_contacts[i].lockN)
		{
			if (m_contacts[i].Fn < B3_FORCE_THRESHOLD)
			{
				b3Draw_draw->DrawPoint(m_x[i], 6.0f, b3Color_yellow);
			}
			else
			{
				b3Draw_draw->DrawPoint(m_x[i], 6.0f, b3Color_red);
			}
		}
		else
		{
			b3Draw_draw->DrawPoint(m_x[i], 6.0f, b3Color_green);
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

		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);
		b3Draw_draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}