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

#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/particle.h>
#include <bounce/cloth/force.h>
#include <bounce/cloth/spring_force.h>
#include <bounce/cloth/cloth_solver.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/collision/collision.h>
#include <bounce/common/draw.h>

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

b3Cloth::b3Cloth(const b3ClothDef& def) :
	m_particleBlocks(sizeof(b3Particle)),
	m_particleContactBlocks(sizeof(b3ParticleContact))
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_mesh = def.mesh;
	m_density = def.density;

	const b3ClothMesh* m = m_mesh;

	m_vertexParticles = (b3Particle**)b3Alloc(m->vertexCount * sizeof(b3Particle*));

	// Create particles
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3ParticleDef pd;
		pd.type = e_dynamicParticle;
		pd.position = m->vertices[i];

		b3Particle* p = CreateParticle(pd);

		p->m_vertex = i;

		m_vertexParticles[i] = p;
	}

	// Compute mass
	ComputeMass();

	// Create forces
	b3StackAllocator* allocator = &m_stackAllocator;

	// Worst-case edge memory
	u32 edgeCount = 3 * m->triangleCount;

	b3UniqueEdge* uniqueEdges = (b3UniqueEdge*)allocator->Allocate(edgeCount * sizeof(b3UniqueEdge));
	u32 uniqueCount = b3FindUniqueEdges(uniqueEdges, m);

	b3SharedEdge* sharedEdges = (b3SharedEdge*)allocator->Allocate(edgeCount * sizeof(b3SharedEdge));
	u32 sharedCount = b3FindSharedEdges(sharedEdges, m);

	for (u32 i = 0; i < uniqueCount; ++i)
	{
		b3UniqueEdge* e = uniqueEdges + i;

		b3Particle* p1 = m_vertexParticles[e->v1];
		b3Particle* p2 = m_vertexParticles[e->v2];

		b3SpringForceDef fd;
		fd.Initialize(p1, p2, def.structural, def.damping);

		CreateForce(fd);
	}

	// Bending
	for (u32 i = 0; i < sharedCount; ++i)
	{
		b3SharedEdge* e = sharedEdges + i;

		b3Particle* p1 = m_vertexParticles[e->v1];
		b3Particle* p2 = m_vertexParticles[e->v2];
		b3Particle* p3 = m_vertexParticles[e->nsv1];
		b3Particle* p4 = m_vertexParticles[e->nsv2];

		b3SpringForceDef fd;
		fd.Initialize(p3, p4, def.bending, def.damping);

		CreateForce(fd);
	}

	allocator->Free(sharedEdges);
	allocator->Free(uniqueEdges);

	// Sewing
	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* line = m->sewingLines + i;

		b3Particle* p1 = m_vertexParticles[line->v1];
		b3Particle* p2 = m_vertexParticles[line->v2];

		b3SpringForceDef fd;
		fd.Initialize(p1, p2, def.structural, def.damping);

		CreateForce(fd);
	}

	m_gravity.SetZero();
	m_world = nullptr;
}

b3Cloth::~b3Cloth()
{
	b3Particle* p = m_particleList.m_head;
	while (p)
	{
		b3Particle* p0 = p;
		p = p->m_next;
		p0->~b3Particle();
	}

	b3Free(m_vertexParticles);

	b3Force* f = m_forceList.m_head;
	while (f)
	{
		b3Force* f0 = f;
		f = f->m_next;
		f0->~b3Force();
		b3Free(f0);
	}
}

b3Particle* b3Cloth::CreateParticle(const b3ParticleDef& def)
{
	void* mem = m_particleBlocks.Allocate();
	b3Particle* p = new(mem) b3Particle(def, this);
	m_particleList.PushFront(p);
	return p;
}

void b3Cloth::DestroyParticle(b3Particle* particle)
{
	if (particle->m_vertex != ~0)
	{
		m_vertexParticles[particle->m_vertex] = NULL;
	}

	m_particleList.Remove(particle);
	particle->~b3Particle();
	m_particleBlocks.Free(particle);
}

b3Force* b3Cloth::CreateForce(const b3ForceDef& def)
{
	b3Force* f = b3Force::Create(&def);
	m_forceList.PushFront(f);
	return f;
}

void b3Cloth::DestroyForce(b3Force* force)
{
	m_forceList.Remove(force);
	b3Force::Destroy(force);
}

float32 b3Cloth::GetEnergy() const
{
	float32 E = 0.0f;
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		E += p->m_mass * b3Dot(p->m_velocity, p->m_velocity);
	}
	return 0.5f * E;
}

b3Particle* b3Cloth::GetVertexParticle(u32 i)
{
	B3_ASSERT(i < m_mesh->vertexCount);
	return m_vertexParticles[i];
}

void b3Cloth::ComputeMass()
{
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_mass = 0.0f;
		p->m_invMass = 0.0f;
	}

	const float32 inv3 = 1.0f / 3.0f;
	const float32 rho = m_density;

	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3ClothMeshTriangle* triangle = m_mesh->triangles + i;

		b3Vec3 v1 = m_mesh->vertices[triangle->v1];
		b3Vec3 v2 = m_mesh->vertices[triangle->v2];
		b3Vec3 v3 = m_mesh->vertices[triangle->v3];

		float32 area = b3Area(v1, v2, v3);
		B3_ASSERT(area > B3_EPSILON);

		float32 mass = rho * area;

		b3Particle* p1 = m_vertexParticles[triangle->v1];
		b3Particle* p2 = m_vertexParticles[triangle->v2];
		b3Particle* p3 = m_vertexParticles[triangle->v3];

		p1->m_mass += inv3 * mass;
		p2->m_mass += inv3 * mass;
		p3->m_mass += inv3 * mass;
	}

	// Invert
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		B3_ASSERT(p->m_mass > 0.0f);
		p->m_invMass = 1.0f / p->m_mass;
	}
}

bool b3Cloth::RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	u32 triangle0 = ~0;
	b3RayCastOutput output0;
	output0.fraction = B3_MAX_FLOAT;

	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3RayCastOutput subOutput;
		if (RayCast(&subOutput, &input, i))
		{
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = i;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}
	}

	if (triangle0 != ~0)
	{
		output->triangle = triangle0;
		output->fraction = output0.fraction;
		output->normal = output0.normal;

		return true;
	}

	return false;
}

bool b3Cloth::RayCast(b3RayCastOutput* output, const b3RayCastInput* input, u32 triangleIndex) const
{
	B3_ASSERT(triangleIndex < m_mesh->triangleCount);
	b3ClothMeshTriangle* triangle = m_mesh->triangles + triangleIndex;

	b3Vec3 v1 = m_vertexParticles[triangle->v1]->m_position;
	b3Vec3 v2 = m_vertexParticles[triangle->v2]->m_position;
	b3Vec3 v3 = m_vertexParticles[triangle->v3]->m_position;

	b3Vec3 p1 = input->p1;
	b3Vec3 p2 = input->p2;
	float32 maxFraction = input->maxFraction;
	b3Vec3 d = p2 - p1;
	B3_ASSERT(b3LengthSquared(d) > B3_EPSILON * B3_EPSILON);

	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	float32 len = b3Length(n);
	if (len <= B3_EPSILON)
	{
		return false;
	}

	n /= len;

	float32 numerator = b3Dot(n, v1 - p1);
	float32 denominator = b3Dot(n, d);

	if (denominator == 0.0f)
	{
		return false;
	}

	float32 fraction = numerator / denominator;

	// Is the intersection not on the segment?
	if (fraction < 0.0f || maxFraction < fraction)
	{
		return false;
	}

	b3Vec3 Q = p1 + fraction * d;

	b3Vec3 A = v1;
	b3Vec3 B = v2;
	b3Vec3 C = v3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	// Barycentric coordinates for Q
	float32 u = b3Dot(QB_x_QC, AB_x_AC);
	float32 v = b3Dot(QC_x_QA, AB_x_AC);
	float32 w = b3Dot(QA_x_QB, AB_x_AC);

	// Is the intersection on the triangle?
	if (u >= 0.0f && v >= 0.0f && w >= 0.0f)
	{
		output->fraction = fraction;

		// Does the ray start from below or above the triangle?
		if (numerator > 0.0f)
		{
			output->normal = -n;
		}
		else
		{
			output->normal = n;
		}

		return true;
	}

	return false;
}

void b3Cloth::UpdateBodyContacts()
{
	// Is there a world attached to this cloth?
	if (m_world == nullptr)
	{
		return;
	}

	B3_PROFILE("Cloth Update Body Contacts");
	
	// Create contacts 
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		b3Sphere s1;
		s1.vertex = p->m_position;
		s1.radius = p->m_radius;

		// Find the deepest penetration
		b3Shape* bestShape = nullptr;
		float32 bestSeparation = 0.0f;
		b3Vec3 bestPoint(0.0f, 0.0f, 0.0f);
		b3Vec3 bestNormal(0.0f, 0.0f, 0.0f);

		for (b3Body* body = m_world->GetBodyList().m_head; body; body = body->GetNext())
		{
			if (p->m_type != e_dynamicParticle)
			{
				continue;
			}

			if (body->GetType() != e_staticBody)
			{
				continue;
			}

			b3Transform xf = body->GetTransform();
			for (b3Shape* shape = body->GetShapeList().m_head; shape; shape = shape->GetNext())
			{
				b3TestSphereOutput output;
				if (shape->TestSphere(&output, s1, xf))
				{
					if (output.separation < bestSeparation)
					{
						bestShape = shape;
						bestSeparation = output.separation;
						bestPoint = output.point;
						bestNormal = output.normal;
					}
				}
			}
		}

		if (bestShape == nullptr)
		{
			p->m_bodyContact.active = false;
			continue;
		}

		// Ensure the the normal points from the particle 1 to shape 2
		b3Shape* shape = bestShape;
		b3Body* body = shape->GetBody();
		float32 separation = bestSeparation;
		b3Vec3 point = bestPoint;
		b3Vec3 normal = -bestNormal;

		b3BodyContact* c = &p->m_bodyContact;
		
		b3BodyContact c0 = *c;

		c->active = true;
		c->p1 = p;
		c->s2 = shape;
		c->s = separation;
		c->p = point;
		c->n = normal;
		c->fn0 = 0.0f;
		c->fn = 0.0f;
		c->ft1 = 0.0f;
		c->ft2 = 0.0f;
		c->nActive = true;
		c->t1Active = false;
		c->t2Active = false;
		c->localPoint1.SetZero();
		c->localPoint2 = body->GetLocalPoint(point);
		c->t1 = b3Perp(normal);
		c->t2 = b3Cross(c->t1, normal);
		c->normalImpulse = 0.0f;
		c->tangentImpulse.SetZero();

#if 0
		// Apply position correction
		p->m_translation += separation * normal;
#endif

		// Update contact state
		if (c0.active == true)
		{
			if (c0.nActive == true)
			{
				c->fn0 = c0.fn0;
				c->fn = c0.fn;
				c->ft1 = c0.ft1;
				c->ft2 = c0.ft2;

				c->normalImpulse = c0.normalImpulse;
				c->tangentImpulse = c0.tangentImpulse;
#if 0
				const float32 kForceTol = 0.0f;

				// Allow the contact to release when the constraint force 
				// switches from a repulsive force to an attractive one.
				// if (c0.fn0 < kForceTol && c0.fn > kForceTol)
				if (c0.fn > kForceTol)
				{
					// Contact force is attractive.
					c->nActive = false;
					continue;
				}
#endif
			}
		}
#if 0
		if (c0.active == false)
		{
			continue;
		}
		
		// A friction force requires an associated normal force.
		if (c0.nActive == false)
		{
			continue;
		}
		
		b3Vec3 v1; v1.SetZero();
		b3Vec3 v2 = p->m_velocity;
		b3Vec3 dv = v2 - v1;

		const float32 kVelTol = 2.0f * B3_EPSILON;

		// Lock particle on surface
		float32 dvt1 = b3Dot(dv, c->t1);
		if (dvt1 * dvt1 < kVelTol * kVelTol)
		{
			c->t1Active = true;
		}

		float32 dvt2 = b3Dot(dv, c->t2);
		if (dvt2 * dvt2 < kVelTol * kVelTol)
		{
			c->t2Active = true;
		}

		// Unlock particle off surface
		float32 normalForce = c->fn;
		
		float32 friction = shape->GetFriction();
		float32 maxFrictionForce = friction * normalForce;

		if (c0.t1Active == true)
		{
			float32 tangentForce1 = c0.ft1;
			if (tangentForce1 * tangentForce1 > maxFrictionForce * maxFrictionForce)
			{
				c->t1Active = false;
			}
		}

		if (c0.t2Active == true)
		{
			float32 tangentForce2 = c0.ft2;
			if (tangentForce2 * tangentForce2 > maxFrictionForce* maxFrictionForce)
			{
				c->t2Active = false;
			}
		}
#endif
	}
}

void b3Cloth::UpdateParticleContacts()
{
	B3_PROFILE("Cloth Update Particle Contacts");

	// Clear buffer
	b3ParticleContact* c = m_particleContactList.m_head;
	while (c)
	{
		b3ParticleContact* c0 = c;
		c = c->m_next;
		m_particleContactList.Remove(c0);
		c0->~b3ParticleContact();
		m_particleContactBlocks.Free(c0);
	}

	// Create particle contacts
	for (b3Particle* p1 = m_particleList.m_head; p1; p1 = p1->m_next)
	{
		for (b3Particle* p2 = p1->m_next; p2; p2 = p2->m_next)
		{
			if (p1->m_type != e_dynamicParticle && p2->m_type != e_dynamicBody)
			{
				// At least one particle should be kinematic or dynamic.
				continue;
			}

			b3Vec3 c1 = p1->m_position;
			float32 r1 = p1->m_radius;

			b3Vec3 c2 = p2->m_position;
			float32 r2 = p2->m_radius;

			b3Vec3 d = c2 - c1;
			float32 dd = b3Dot(d, d);
			float32 totalRadius = r1 + r2;
			if (dd > totalRadius * totalRadius)
			{
				continue;
			}

			b3Vec3 n(0.0f, 1.0f, 0.0f);
			if (dd > B3_EPSILON * B3_EPSILON)
			{
				float32 distance = b3Sqrt(dd);
				n = d / distance;
			}

			b3ParticleContact* c = (b3ParticleContact*)m_particleContactBlocks.Allocate();
			c->p1 = p1;
			c->p2 = p2;
			c->normalImpulse = 0.0f;
			c->t1 = b3Perp(n);
			c->t2 = b3Cross(c->t1, n);
			c->tangentImpulse.SetZero();

			m_particleContactList.PushFront(c);
		}
	}
}

void b3Cloth::Solve(float32 dt, const b3Vec3& gravity)
{
	B3_PROFILE("Cloth Solve");

	// Solve
	b3ClothSolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.bodyContactCapacity = m_particleList.m_count;
	solverDef.particleContactCapacity = m_particleContactList.m_count;

	b3ClothSolver solver(solverDef);

	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_bodyContact.active)
		{
			solver.Add(&p->m_bodyContact);
		}
	}

	for (b3ParticleContact* c = m_particleContactList.m_head; c; c = c->m_next)
	{
		solver.Add(c);
	}

	// Solve	
	solver.Solve(dt, gravity);
}

void b3Cloth::UpdateContacts()
{
	// Update body contacts
	UpdateBodyContacts();

#if 0
	// Update particle contacts
	UpdateParticleContacts();
#endif
}

void b3Cloth::Step(float32 dt)
{
	B3_PROFILE("Cloth Step");

	// Update contacts
	UpdateContacts();

	// Solve constraints, integrate state, clear forces and translations. 
	if (dt > 0.0f)
	{
		Solve(dt, m_gravity);
	}

	// Clear external applied forces and translations
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}
}

void b3Cloth::Draw() const
{
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_type == e_staticParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0f, b3Color_white);
		}

		if (p->m_type == e_kinematicParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0f, b3Color_blue);
		}

		if (p->m_type == e_dynamicParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0f, b3Color_green);
		}
	}

	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		if (f->m_type == e_springForce)
		{
			b3SpringForce* s = (b3SpringForce*)f;
			b3Particle* p1 = s->m_p1;
			b3Particle* p2 = s->m_p2;

			b3Draw_draw->DrawSegment(p1->m_position, p2->m_position, b3Color_black);
		}
	}

	const b3ClothMesh* m = m_mesh;

	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* s = m->sewingLines + i;
		b3Particle* p1 = m_vertexParticles[s->v1];
		b3Particle* p2 = m_vertexParticles[s->v2];

		b3Draw_draw->DrawSegment(p1->m_position, p2->m_position, b3Color_white);
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t = m->triangles + i;

		b3Particle* p1 = m_vertexParticles[t->v1];
		b3Particle* p2 = m_vertexParticles[t->v2];
		b3Particle* p3 = m_vertexParticles[t->v3];

		b3Vec3 v1 = p1->m_position;
		b3Vec3 v2 = p2->m_position;
		b3Vec3 v3 = p3->m_position;

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		b3Vec3 n2 = -n1;
		b3Draw_draw->DrawSolidTriangle(n2, v1, v3, v2, b3Color_blue);
	}
}