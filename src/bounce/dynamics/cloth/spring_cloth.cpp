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
#include <bounce/dynamics/cloth/cloth_mesh.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/draw.h>

#define B3_FORCE_THRESHOLD 0.005f

#define B3_CLOTH_BENDING 0

#define B3_CLOTH_FRICTION 1

b3SpringCloth::b3SpringCloth()
{
	m_allocator = nullptr;

	m_mesh = nullptr;

	m_gravity.SetZero();

	m_x = nullptr;
	m_v = nullptr;
	m_f = nullptr;
	m_y = nullptr;
	m_z = nullptr;
	m_x0 = nullptr;
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
	b3Free(m_z);
	b3Free(m_x0);
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

static u32 b3FindUniqueEdges(b3UniqueEdge* uniqueEdges, const b3ClothMesh* m)
{
	u32 uniqueCount = 0;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t1 = m->triangles + i;
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

	return uniqueCount;
}

struct b3SharedEdge
{
	u32 v1, v2;
	u32 nsv1, nsv2;
};

static u32 b3FindSharedEdges(b3SharedEdge* sharedEdges, const b3ClothMesh* m)
{
	u32 sharedCount = 0;

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t1 = m->triangles + i;
		u32 i1s[3] = { t1->v1, t1->v2, t1->v3 };

		for (u32 j1 = 0; j1 < 3; ++j1)
		{
			u32 k1 = j1 + 1 < 3 ? j1 + 1 : 0;

			u32 t1v1 = i1s[j1];
			u32 t1v2 = i1s[k1];

			for (u32 j = i + 1; j < m->triangleCount; ++j)
			{
				b3ClothMeshTriangle* t2 = m->triangles + j;
				u32 i2s[3] = { t2->v1, t2->v2, t2->v3 };

				for (u32 j2 = 0; j2 < 3; ++j2)
				{
					u32 k2 = j2 + 1 < 3 ? j2 + 1 : 0;

					u32 t2v1 = i2s[j2];
					u32 t2v2 = i2s[k2];

					if (t1v1 == t2v2 && t1v2 == t2v1)
					{
						// The triangles are adjacent.
						u32 k3 = k1 + 1 < 3 ? k1 + 1 : 0;
						u32 t1v3 = i1s[k3];

						u32 k4 = k2 + 1 < 3 ? k2 + 1 : 0;
						u32 t2v3 = i2s[k4];

						// Add shared edge and non-shared vertices.
						b3SharedEdge se;
						se.v1 = t1v1;
						se.v2 = t1v2;
						se.nsv1 = t1v3;
						se.nsv2 = t2v3;

						sharedEdges[sharedCount++] = se;

						break;
					}
				}
			}
		}
	}

	return sharedCount;
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

	const b3ClothMesh* m = m_mesh;

	m_massCount = m->vertexCount;
	m_x = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_v = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_f = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_inv_m = (float32*)b3Alloc(m_massCount * sizeof(float32));
	m_y = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_z = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
	m_x0 = (b3Vec3*)b3Alloc(m_massCount * sizeof(b3Vec3));
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
		m_z[i].SetZero();
		m_x0[i].SetZero();
		m_types[i] = b3MassType::e_staticMass;
	}

	// Initialize mass
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t = m->triangles + i;

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

	b3UniqueEdge* uniqueEdges = (b3UniqueEdge*)m_allocator->Allocate(edgeCount * sizeof(b3UniqueEdge));
	u32 uniqueCount = b3FindUniqueEdges(uniqueEdges, m);

	u32 springCapacity = uniqueCount;
	
#if B3_CLOTH_BENDING
	
	b3SharedEdge* sharedEdges = (b3SharedEdge*)m_allocator->Allocate(edgeCount * sizeof(b3SharedEdge));
	u32 sharedCount = b3FindSharedEdges(sharedEdges, m);

	springCapacity += sharedCount;

#endif
	
	springCapacity += m->sewingLineCount;

	m_springs = (b3Spring*)b3Alloc(springCapacity * sizeof(b3Spring));

	// Tension
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
		
		B3_ASSERT(S->L0 > B3_EPSILON);

		S->ks = def.ks;
		S->kd = def.kd;
		++m_springCount;
	}

#if B3_CLOTH_BENDING

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

	m_allocator->Free(sharedEdges);
#endif

	m_allocator->Free(uniqueEdges);

	// Sewing
	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* line = m->sewingLines + i;

		b3Spring* S = m_springs + m_springCount;
		S->type = e_strechSpring;
		S->i1 = line->v1;
		S->i2 = line->v2;
		S->L0 = 0.0f;
		S->ks = def.ks;
		S->kd = def.kd;
		++m_springCount;
	}

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

		// Streching
		b3Vec3 dx = x1 - x2;
		float32 L = b3Length(dx);

		if (L >= L0)
		{
			// Force is tension.
			b3Vec3 n = dx / L;

			b3Vec3 sf1 = -ks * (L - L0) * n;
			b3Vec3 sf2 = -sf1;

			T[i1] += sf1;
			T[i2] += sf2;
		}

		// Damping
		b3Vec3 dv = v1 - v2;

		b3Vec3 df1 = -kd * dv;
		b3Vec3 df2 = -df1;

		T[i1] += df1;
		T[i2] += df2;
	}

	for (u32 i = 0; i < T.Count(); ++i)
	{
		if (m_types[i] != b3MassType::e_dynamicMass)
		{
			T[i].SetZero();
		}
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

		// Save the old contact
		b3MassContact c0 = *c;

		// Create a new contact
		c->lockN = false;
		c->lockT1 = false;
		c->lockT2 = false;

		b3Sphere s1;
		s1.vertex = m_x[i];
		s1.radius = m_r;

		// Find the deepest penetration
		float32 bestSeparation = 0.0f;
		b3Vec3 bestNormal(0.0f, 0.0f, 0.0f);
		u32 bestIndex = ~0;

		for (u32 j = 0; j < m_shapeCount; ++j)
		{
			b3Shape* s2 = m_shapes[j];

			b3Transform xf2;
			xf2.SetIdentity();

			b3TestSphereOutput output;
			if (s2->TestSphere(&output, s1, xf2) == false)
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

		if (bestIndex != ~0)
		{
			B3_ASSERT(bestSeparation <= 0.0f);

			b3Shape* shape = m_shapes[bestIndex];
			float32 s = bestSeparation;
			b3Vec3 n = bestNormal;

			// Update contact manifold
			// Remember the normal orientation is from shape 2 to shape 1 (mass)
			c->j = bestIndex;
			c->n = n;
			c->t1 = b3Perp(n);
			c->t2 = b3Cross(c->t1, n);
			c->lockN = true;

			// Apply position correction
			m_y[i] -= s * n;
		}

		// Update contact state
		if (c0.lockN == true && c->lockN == true)
		{
			// The contact persists
			
			// Has the contact constraint been satisfied?
			if (c0.Fn <= -B3_FORCE_THRESHOLD)
			{
				// Contact force is attractive.

				// Terminate the contact.
				c->lockN = false;
			}
		}

#if 0
		// Notify the new contact state
		if (wasLockedN == false && c->lockN == true)
		{
			// The contact has begun
		}

		if (wasLockedN == true && c->lockN == false)
		{
			// The contact has ended
		}
#endif
		if (c->lockN == false)
		{
			continue;
		}

#if B3_CLOTH_FRICTION == 1

		// A friction force requires an associated normal force.
		if (c0.lockN == false)
		{
			continue;
		}

		b3Shape* s = m_shapes[c->j];
		b3Vec3 n = c->n;
		float32 u = s->GetFriction();
		float32 normalForce = c0.Fn;

		// Relative velocity
		b3Vec3 dv = m_v[i];

		b3Vec3 t1 = dv - b3Dot(dv, n) * n;
		if (b3Dot(t1, t1) > B3_EPSILON * B3_EPSILON)
		{
			// Create a dynamic basis
			t1.Normalize();
			
			b3Vec3 t2 = b3Cross(t1, n);
			t2.Normalize();

			c->t1 = t1;
			c->t2 = t2;
		}
		else
		{
			c->lockT1 = true;
			c->lockT2 = true;
			continue;
		}

		b3Vec3 ts[2];
		ts[0] = c->t1;
		ts[1] = c->t2;

		bool lockT[2];
		lockT[0] = c->lockT1;
		lockT[1] = c->lockT2;

		bool lockT0[2];
		lockT0[0] = c0.lockT1;
		lockT0[1] = c0.lockT2;

		float32 Ft0[2];
		Ft0[0] = c0.Ft1;
		Ft0[1] = c0.Ft2;

		for (u32 k = 0; k < 2; ++k)
		{
			b3Vec3 t = ts[k];

			// Relative tangential velocity
			float32 dvt = b3Dot(dv, t);

			if (dvt * dvt <= B3_EPSILON * B3_EPSILON)
			{
				// Lock mass on surface
				lockT[k] = true;
			}

			if (lockT0[k] == true && lockT[k] == true)
			{
				// The contact persists
				float32 maxForce = u * normalForce;
				
				if (Ft0[k] * Ft0[k] > maxForce * maxForce)
				{
					// Unlock mass off surface
					lockT[k] = false;
				}
			}
		}

		c->lockT1 = lockT[0];
		c->lockT2 = lockT[1];

#endif

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

	// Apply weights
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
		b3MassContact* c = m_contacts + i;

		if (c->lockN == false)
		{
			continue;
		}

		b3Vec3 force = forces[i];

		// Signed normal force magnitude
		c->Fn = b3Dot(force, c->n);

		// Signed tangent force magnitude
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
	const b3ClothMesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		if (m_types[i] == b3MassType::e_staticMass)
		{
			b3Draw_draw->DrawPoint(m_x[i], 4.0f, b3Color_white);
		}
		
		if (m_types[i] == b3MassType::e_kinematicMass)
		{
			b3Draw_draw->DrawPoint(m_x[i], 4.0f, b3Color_blue);
		}
		
		if (m_types[i] == b3MassType::e_dynamicMass)
		{
			b3Draw_draw->DrawPoint(m_x[i], 4.0f, b3Color_green);
		}

		b3MassContact* c = m_contacts + i;
		
		if (c->lockN)
		{
			b3Draw_draw->DrawSegment(m_x[i], m_x[i] + c->n, b3Color_yellow);
		}
	}

	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* s = m_springs + i;

		b3Vec3 x1 = m_x[s->i1];
		b3Vec3 x2 = m_x[s->i2];
		
		if (s->type == e_strechSpring)
		{
			b3Draw_draw->DrawSegment(x1, x2, b3Color_black);
		}
	}
	
	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* s = m->sewingLines + i;

		b3Vec3 x1 = m_x[s->v1];
		b3Vec3 x2 = m_x[s->v2];

		b3Draw_draw->DrawSegment(x1, x2, b3Color_white);
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t = m->triangles + i;

		b3Vec3 v1 = m_x[t->v1];
		b3Vec3 v2 = m_x[t->v2];
		b3Vec3 v3 = m_x[t->v3];

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		b3Vec3 n2 = -n1;
		b3Draw_draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}