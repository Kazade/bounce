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

#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/sparse_mat33.h>
#include <bounce/dynamics/cloth/cloth_mesh.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/draw.h>

#define B3_FORCE_THRESHOLD 0.005f

#define B3_CLOTH_BENDING 0

#define B3_CLOTH_FRICTION 0

b3Cloth::b3Cloth()
{
	m_gravity.SetZero();

	m_particleCount = 0;
	m_particles = nullptr;

	m_springs = nullptr;
	m_springCount = 0;

	m_contacts = nullptr;
	//m_contactCount = 0;

	m_shapeCount = 0;

	m_mesh = nullptr;

	m_gravity.SetZero();
}

b3Cloth::~b3Cloth()
{
	b3Free(m_particles);
	b3Free(m_springs);
	b3Free(m_contacts);
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

void b3Cloth::Initialize(const b3ClothDef& def)
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_mesh = def.mesh;
	m_density = def.density;

	const b3ClothMesh* m = m_mesh;

	m_particleCount = m->vertexCount;
	m_particles = (b3Particle*)b3Alloc(m_particleCount * sizeof(b3Particle));
	m_contacts = (b3ParticleContact*)b3Alloc(m_particleCount * sizeof(b3ParticleContact));

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles + i;	
		p->type = e_dynamicParticle;
		p->position = m->vertices[i];
		p->velocity.SetZero();
		p->force.SetZero();
		p->mass = 0.0f;
		p->invMass = 0.0f;
		p->radius = def.r;
		p->userData = nullptr;

		p->translation.SetZero();
		p->x.SetZero();
		p->tension.SetZero();

		b3ParticleContact* c = m_contacts + i;
		c->n_active = false;
		c->t1_active = false;
		c->t2_active = false;
	}

	// Compute mass
	ResetMass();

	// Initialize springs
	u32 edgeCount = 3 * m->triangleCount;

	b3UniqueEdge* uniqueEdges = (b3UniqueEdge*)m_allocator.Allocate(edgeCount * sizeof(b3UniqueEdge));
	u32 uniqueCount = b3FindUniqueEdges(uniqueEdges, m);

	u32 springCapacity = uniqueCount;
	
#if B3_CLOTH_BENDING
	
	b3SharedEdge* sharedEdges = (b3SharedEdge*)m_allocator.Allocate(edgeCount * sizeof(b3SharedEdge));
	u32 sharedCount = b3FindSharedEdges(sharedEdges, m);

	springCapacity += sharedCount;

#endif
	
	springCapacity += m->sewingLineCount;

	m_springs = (b3Spring*)b3Alloc(springCapacity * sizeof(b3Spring));

	// Tension
	for (u32 i = 0; i < uniqueCount; ++i)
	{
		b3UniqueEdge* e = uniqueEdges + i;

		b3Vec3 v1 = m->vertices[e->v1];
		b3Vec3 v2 = m->vertices[e->v2];

		b3Particle* p1 = m_particles + e->v1;
		b3Particle* p2 = m_particles + e->v2;

		b3Spring* s = m_springs + m_springCount++;
		s->type = e_strechSpring;
		s->p1 = p1;
		s->p2 = p2;
		s->L0 = b3Distance(p1->position, p2->position);
		s->ks = def.ks;
		s->kd = def.kd;
	}

#if B3_CLOTH_BENDING

	// Bending
	for (u32 i = 0; i < sharedCount; ++i)
	{
		b3SharedEdge* e = sharedEdges + i;

		b3Vec3 v1 = m->vertices[e->nsv1];
		b3Vec3 v2 = m->vertices[e->nsv2];

		b3Particle* p1 = m_particles + e->nsv1;
		b3Particle* p2 = m_particles + e->nsv2;

		b3Spring* s = m_springs + m_springCount++;
		s->type = e_bendSpring;
		s->p1 = p1;
		s->p2 = p2;
		s->L0 = b3Distance(p1->position, p2->position);
		s->ks = def.kb;
		s->kd = def.kd;
	}

	m_allocator.Free(sharedEdges);
#endif

	m_allocator.Free(uniqueEdges);

	// Sewing
	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* line = m->sewingLines + i;
		
		b3Particle* p1 = m_particles + line->v1;
		b3Particle* p2 = m_particles + line->v2;

		b3Spring* s = m_springs + m_springCount++;
		s->type = e_strechSpring;
		s->p1 = p1;
		s->p2 = p2;
		s->L0 = 0.0f;
		s->ks = def.ks;
		s->kd = def.kd;
	}

	B3_ASSERT(m_springCount <= springCapacity);
}

void b3Cloth::ResetMass()
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i].mass = 0.0f;
		m_particles[i].invMass = 0.0f;
	}

	const float32 inv3 = 1.0f / 3.0f;
	const float32 rho = m_density;

	// Accumulate the mass about the body origin of all triangles.
	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3ClothMeshTriangle* triangle = m_mesh->triangles + i;
		u32 v1 = triangle->v1;
		u32 v2 = triangle->v2;
		u32 v3 = triangle->v3;

		b3Vec3 p1 = m_mesh->vertices[v1];
		b3Vec3 p2 = m_mesh->vertices[v2];
		b3Vec3 p3 = m_mesh->vertices[v3];

		float32 area = b3Area(p1, p2, p3);
		B3_ASSERT(area > B3_EPSILON);

		float32 mass = rho * area;

		m_particles[v1].mass += inv3 * mass;
		m_particles[v2].mass += inv3 * mass;
		m_particles[v3].mass += inv3 * mass;
	}

	// Invert
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		B3_ASSERT(m_particles[i].mass > 0.0f);
		m_particles[i].invMass = 1.0f / m_particles[i].mass;
	}
}

void b3Cloth::AddShape(b3Shape* shape)
{
	B3_ASSERT(m_shapeCount < B3_CLOTH_SHAPE_CAPACITY);

	if (m_shapeCount == B3_CLOTH_SHAPE_CAPACITY)
	{
		return;
	}

	m_shapes[m_shapeCount++] = shape;
}

void b3Cloth::UpdateContacts()
{
	B3_PROFILE("Update Contacts");
	
	// Clear active flags
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_contacts[i].n_active = false;
		m_contacts[i].t1_active = false;
		m_contacts[i].t2_active = false;
	}

	// Create contacts 
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles + i;

		// Static particles can't participate in collisions.
		if (p->type == e_staticParticle)
		{
			continue;
		}

		b3ParticleContact* c = m_contacts + i;

		// Save the old contact
		b3ParticleContact c0 = *c;

		b3Sphere s1;
		s1.vertex = p->position;
		s1.radius = p->radius;

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
			c->n_active = true;
			c->p1 = p;
			c->s2 = shape;
			c->n = n;
			c->t1 = b3Perp(n);
			c->t2 = b3Cross(c->t1, n);

			// Apply position correction
			p->translation -= s * n;
		}

		// Update contact state
		if (c0.n_active == true && c->n_active == true)
		{
			// The contact persists
			
			// Has the contact constraint been satisfied?
			if (c0.Fn <= -B3_FORCE_THRESHOLD)
			{
				// Contact force is attractive.

				// Terminate the contact.
				c->n_active = false;
			}
		}

#if 0
		// Notify the new contact state
		if (c0.n_active == false && c->n_active == true)
		{
			// The contact has begun
		}

		if (c0.n_active == true && c->active_n == false)
		{
			// The contact has ended
		}
#endif
		if (c->n_active == false)
		{
			continue;
		}

#if B3_CLOTH_FRICTION == 1

		// A friction force requires an associated normal force.
		if (c0.n_active == false)
		{
			continue;
		}

		b3Shape* s = c->s;
		b3Vec3 n = c->n;
		float32 u = s->GetFriction();
		float32 normalForce = c0.Fn;

		// Relative velocity
		b3Vec3 dv = p->velocity;

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
			c->t1_active = true;
			c->t2_active = true;
			continue;
		}

		b3Vec3 ts[2];
		ts[0] = c->t1;
		ts[1] = c->t2;

		bool t_active[2];
		t_active[0] = c->t1_active;
		t_active[1] = c->t2_active;

		bool t_active0[2];
		t_active0[0] = c0.t1_active;
		t_active0[1] = c0.t2_active;

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
				// Lock particle on surface
				t_active[k] = true;
			}

			if (t_active0[k] == true && t_active[k] == true)
			{
				// The contact persists
				float32 maxForce = u * normalForce;
				
				if (Ft0[k] * Ft0[k] > maxForce * maxForce)
				{
					// Unlock particle off surface
					t_active[k] = false;
				}
			}
		}

		c->t1_active = t_active[0];
		c->t2_active = t_active[1];

#endif

	}

}

void b3Cloth::Solve(float32 dt)
{
	B3_PROFILE("Solve");

	// Clear tension
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i].tension.SetZero();
	}

	// Solve
	b3ClothSolverDef solverDef;
	solverDef.stack = &m_allocator;
	solverDef.particleCapacity = m_particleCount;
	solverDef.springCapacity = m_springCount;
	solverDef.contactCapacity = m_particleCount;

	b3ClothSolver solver(solverDef);

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		solver.Add(m_particles + i);
	}

	for (u32 i = 0; i < m_springCount; ++i)
	{
		solver.Add(m_springs + i);
	}

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		if (m_contacts[i].n_active)
		{
			solver.Add(m_contacts + i);
		}
	}

	// Solve
	solver.Solve(dt, m_gravity);

	// Clear external applied forces
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i].force.SetZero();
	}

	// Clear translations
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i].translation.SetZero();
	}
}

void b3Cloth::Step(float32 dt)
{
	B3_PROFILE("Step");

	// Update contacts
	UpdateContacts();

	// Solve constraints, integrate state, clear forces and translations. 
	if (dt > 0.0f)
	{
		Solve(dt);
	}
}

void b3Cloth::Apply() const
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_mesh->vertices[i] = m_particles[i].position;
	}
}

void b3Cloth::Draw() const
{
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles + i;

		if (p->type == e_staticParticle)
		{
			b3Draw_draw->DrawPoint(p->position, 4.0f, b3Color_white);
		}
		
		if (p->type == e_kinematicParticle)
		{
			b3Draw_draw->DrawPoint(p->position, 4.0f, b3Color_blue);
		}
		
		if (p->type == e_dynamicParticle)
		{
			b3Draw_draw->DrawPoint(p->position, 4.0f, b3Color_green);
		}

		b3ParticleContact* c = m_contacts + i;
		
		if (c->n_active)
		{
			b3Draw_draw->DrawSegment(p->position, p->position + c->n, b3Color_yellow);
		}
	}

	for (u32 i = 0; i < m_springCount; ++i)
	{
		b3Spring* s = m_springs + i;
		b3Particle* p1 = s->p1;
		b3Particle* p2 = s->p2;

		if (s->type == e_strechSpring)
		{
			b3Draw_draw->DrawSegment(p1->position, p2->position, b3Color_black);
		}
	}
	
	const b3ClothMesh* m = m_mesh;

	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* s = m->sewingLines + i;
		b3Particle* p1 = m_particles + s->v1;
		b3Particle* p2 = m_particles + s->v2;

		b3Draw_draw->DrawSegment(p1->position, p2->position, b3Color_white);
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t = m->triangles + i;

		b3Particle* p1 = m_particles + t->v1;
		b3Particle* p2 = m_particles + t->v2;
		b3Particle* p3 = m_particles + t->v3;

		b3Vec3 v1 = p1->position;
		b3Vec3 v2 = p2->position;
		b3Vec3 v3 = p3->position;

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		b3Vec3 n2 = -n1;
		b3Draw_draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}