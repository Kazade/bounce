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
#include <bounce/cloth/cloth_triangle.h>
#include <bounce/cloth/force.h>
#include <bounce/cloth/strech_force.h>
#include <bounce/cloth/shear_force.h>
#include <bounce/cloth/spring_force.h>
#include <bounce/cloth/cloth_solver.h>
#include <bounce/common/draw.h>

static B3_FORCE_INLINE u32 b3NextIndex(u32 i)
{
	return i + 1 < 3 ? i + 1 : 0;
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
	m_particleBlocks(sizeof(b3Particle))
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_mesh = def.mesh;
	m_density = def.density;
	m_contactManager.m_cloth = this;

	const b3ClothMesh* m = m_mesh;

	// Initialize particles
	m_particles = (b3Particle**)b3Alloc(m->vertexCount * sizeof(b3Particle*));
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3ParticleDef pd;
		pd.type = e_dynamicParticle;
		pd.mass = 1.0f;
		pd.radius = def.thickness;
		pd.friction = def.friction;
		pd.position = m->vertices[i];

		b3Particle* p = CreateParticle(pd);

		p->m_vertex = i;
		m_particles[i] = p;
	}

	// Compute mass
	ComputeMass();

	// Initialize triangles
	m_triangles = (b3ClothTriangle*)b3Alloc(m_mesh->triangleCount * sizeof(b3ClothTriangle));
	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3ClothMeshTriangle* meshTriangle = m_mesh->triangles + i;
		b3ClothTriangle* triangle = m_triangles + i;

		triangle->m_cloth = this;
		triangle->m_radius = def.thickness;
		triangle->m_friction = def.friction;
		triangle->m_triangle = i;

		b3Vec3 A = m_mesh->vertices[meshTriangle->v1];
		b3Vec3 B = m_mesh->vertices[meshTriangle->v2];
		b3Vec3 C = m_mesh->vertices[meshTriangle->v3];

		b3AABB3 aabb;
		aabb.Set(A, B, C);
		aabb.Extend(triangle->m_radius);

		triangle->m_aabbProxy.type = e_triangleProxy;
		triangle->m_aabbProxy.owner = triangle;
		triangle->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, &triangle->m_aabbProxy);

		b3Vec3 AB = B - A;
		b3Vec3 AC = C - A;

		// v1
		b3Vec2 uv1;
		uv1.SetZero();

		// v2
		b3Vec2 uv2;
		uv2.x = b3Length(AB);
		uv2.y = 0.0f;

		// v3
		B3_ASSERT(uv2.x > 0.0f);
		b3Vec3 n_AB = AB / uv2.x;

		// A  = b * h / 2
		// h = (A * 2) / b
		float32 A2 = b3Length(b3Cross(AB, AC));
		B3_ASSERT(A2 > 0.0f);

		b3Vec2 uv3;
		uv3.x = b3Dot(AC, n_AB);
		uv3.y = A2 / uv2.x;

		// Strech matrix
		float32 du1 = uv2.x - uv1.x;
		float32 dv1 = uv2.y - uv1.y;
		float32 du2 = uv3.x - uv1.x;
		float32 dv2 = uv3.y - uv1.y;
		
		triangle->m_du1 = du1;
		triangle->m_dv1 = dv1;
		triangle->m_du2 = du2;
		triangle->m_dv2 = dv2;

		float32 det = du1 * dv2 - du2 * dv1;
		B3_ASSERT(det != 0.0f);
		triangle->m_inv_det = 1.0f / det;

		// Area
		triangle->m_alpha = 0.5f * A2;

		// Create strech force
		b3StrechForceDef sfdef;
		sfdef.triangle = triangle;
		sfdef.streching = def.streching;
		sfdef.damping = def.damping;
		sfdef.bu = 1.0f;
		sfdef.bv = 1.0f;

		if (def.streching > 0.0f)
		{
			CreateForce(sfdef);
		}

		b3ShearForceDef shdef;
		shdef.triangle = triangle;
		shdef.shearing = def.shearing;
		shdef.damping = def.damping;

		if (def.shearing > 0.0f)
		{
			CreateForce(shdef);
		}
	}

	// Initialize forces
	b3StackAllocator* allocator = &m_stackAllocator;

	// Worst-case edge memory
	u32 edgeCount = 3 * m->triangleCount;

	b3SharedEdge* sharedEdges = (b3SharedEdge*)allocator->Allocate(edgeCount * sizeof(b3SharedEdge));
	u32 sharedCount = b3FindSharedEdges(sharedEdges, m);

	// Bending
	for (u32 i = 0; i < sharedCount; ++i)
	{
		b3SharedEdge* e = sharedEdges + i;

		b3Particle* p1 = m_particles[e->v1];
		b3Particle* p2 = m_particles[e->v2];
		b3Particle* p3 = m_particles[e->nsv1];
		b3Particle* p4 = m_particles[e->nsv2];

		b3SpringForceDef fd;
		fd.Initialize(p3, p4, def.bending, def.damping);

		if (def.bending > 0.0f)
		{
			CreateForce(fd);
		}
	}
	
	allocator->Free(sharedEdges);

	// Sewing
	for (u32 i = 0; i < m->sewingLineCount; ++i)
	{
		b3ClothMeshSewingLine* line = m->sewingLines + i;

		b3Particle* p1 = m_particles[line->v1];
		b3Particle* p2 = m_particles[line->v2];

		b3SpringForceDef fd;
		fd.Initialize(p1, p2, def.sewing, def.damping);

		if (def.sewing > 0.0f)
		{
			CreateForce(fd);
		}
	}

	m_gravity.SetZero();
	m_world = nullptr;
}

b3Cloth::~b3Cloth()
{
	b3Free(m_particles);
	b3Free(m_triangles);

	b3Particle* p = m_particleList.m_head;
	while (p)
	{
		b3Particle* p0 = p;
		p = p->m_next;
		p0->~b3Particle();
	}

	b3Force* f = m_forceList.m_head;
	while (f)
	{
		b3Force* f0 = f;
		f = f->m_next;
		b3Force::Destroy(f0);
	}
}

void b3Cloth::SetWorld(b3World* world)
{
	if (!world && m_world)
	{
		// Destroy body contacts
		b3ParticleBodyContact* c = m_contactManager.m_particleBodyContactList.m_head;
		while (c)
		{
			b3ParticleBodyContact* boom = c;
			c = c->m_next;
			m_contactManager.Destroy(boom);
		}
	}

	m_world = world;
}

b3Particle* b3Cloth::CreateParticle(const b3ParticleDef& def)
{
	void* mem = m_particleBlocks.Allocate();
	b3Particle* p = new(mem) b3Particle(def, this);

	b3AABB3 aabb;
	aabb.Set(p->m_position, p->m_radius);

	p->m_aabbProxy.type = e_particleProxy;
	p->m_aabbProxy.owner = p;
	p->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, &p->m_aabbProxy);

	m_particleList.PushFront(p);

	return p;
}

void b3Cloth::DestroyParticle(b3Particle* particle)
{
	B3_ASSERT(particle->m_vertex == ~0);
	
	// Destroy particle forces
	b3Force* f = m_forceList.m_head;
	while (f)
	{
		b3Force* f0 = f;
		f = f->m_next;
		
		if (f0->HasParticle(particle))
		{
			m_forceList.Remove(f0);
			b3Force::Destroy(f0);
		}
	}

	// Destroy particle contacts
	particle->DestroyContacts();

	// Destroy AABB proxy
	m_contactManager.m_broadPhase.DestroyProxy(particle->m_broadPhaseId);

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

b3Particle* b3Cloth::GetParticle(u32 i)
{
	B3_ASSERT(i < m_mesh->vertexCount);
	return m_particles[i];
}

b3ClothTriangle* b3Cloth::GetTriangle(u32 i)
{
	B3_ASSERT(i < m_mesh->triangleCount);
	return m_triangles + i;
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
		B3_ASSERT(area > 0.0f);

		float32 mass = rho * area;

		b3Particle* p1 = m_particles[triangle->v1];
		b3Particle* p2 = m_particles[triangle->v2];
		b3Particle* p3 = m_particles[triangle->v3];

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

struct b3ClothRayCastSingleCallback
{
	float32 Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get primitive associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3ClothAABBProxy* proxy = (b3ClothAABBProxy*)userData;

		if (proxy->type != e_triangleProxy)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3ClothTriangle* triangle = (b3ClothTriangle*)proxy->owner;
		u32 triangleIndex = triangle->GetTriangle();

		b3RayCastOutput subOutput;
		if (cloth->RayCast(&subOutput, &input, triangleIndex))
		{
			// Ray hits triangle.
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = triangleIndex;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	const b3Cloth* cloth;
	const b3BroadPhase* broadPhase;
	u32 triangle0;
	b3RayCastOutput output0;
};

bool b3Cloth::RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	b3ClothRayCastSingleCallback callback;
	callback.cloth = this;
	callback.broadPhase = &m_contactManager.m_broadPhase;
	callback.triangle0 = ~0;
	callback.output0.fraction = B3_MAX_FLOAT;

	m_contactManager.m_broadPhase.RayCast(&callback, input);

	if (callback.triangle0 != ~0)
	{
		output->triangle = callback.triangle0;
		output->fraction = callback.output0.fraction;
		output->normal = callback.output0.normal;

		return true;
	}

	return false;
}

bool b3Cloth::RayCast(b3RayCastOutput* output, const b3RayCastInput* input, u32 triangleIndex) const
{
	B3_ASSERT(triangleIndex < m_mesh->triangleCount);
	b3ClothMeshTriangle* triangle = m_mesh->triangles + triangleIndex;

	b3Vec3 v1 = m_particles[triangle->v1]->m_position;
	b3Vec3 v2 = m_particles[triangle->v2]->m_position;
	b3Vec3 v3 = m_particles[triangle->v3]->m_position;

	return b3RayCast(output, input, v1, v2, v3);
}

void b3Cloth::Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Cloth Solve");

	// Solve
	b3ClothSolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.bodyContactCapacity = m_contactManager.m_particleBodyContactList.m_count;
	solverDef.triangleContactCapacity = m_contactManager.m_particleTriangleContactList.m_count;

	b3ClothSolver solver(solverDef);

	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3ParticleTriangleContact* c = m_contactManager.m_particleTriangleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	for (b3ParticleBodyContact* c = m_contactManager.m_particleBodyContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}
	
	// Solve	
	solver.Solve(dt, gravity, velocityIterations, positionIterations);
}

void b3Cloth::Step(float32 dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Cloth Step");

	// Update contacts
	m_contactManager.UpdateContacts();

	// Integrate state, solve constraints. 
	if (dt > 0.0f)
	{
		Solve(dt, m_gravity, velocityIterations, positionIterations);
	}

	// Clear external applied forces and translations
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}

	// Synchronize particles
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		b3Vec3 displacement = dt * p->m_velocity;

		p->Synchronize(displacement);
	}

	// Synchronize triangles
	for (u32 i = 0; i < m_mesh->triangleCount; ++i)
	{
		b3ClothMeshTriangle* triangle = m_mesh->triangles + i;

		b3Particle* p1 = m_particles[triangle->v1];
		b3Particle* p2 = m_particles[triangle->v2];
		b3Particle* p3 = m_particles[triangle->v3];

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;
		b3Vec3 v3 = p3->m_velocity;

		b3Vec3 velocity = (v1 + v2 + v3) / 3.0f;

		b3Vec3 displacement = dt * velocity;

		m_triangles[i].Synchronize(displacement);
	}

	// Find new contacts
	m_contactManager.FindNewContacts();
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
		b3Particle* p1 = m_particles[s->v1];
		b3Particle* p2 = m_particles[s->v2];

		b3Draw_draw->DrawSegment(p1->m_position, p2->m_position, b3Color_white);
	}

	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* t = m->triangles + i;

		b3Particle* p1 = m_particles[t->v1];
		b3Particle* p2 = m_particles[t->v2];
		b3Particle* p3 = m_particles[t->v3];

		b3Vec3 v1 = p1->m_position;
		b3Vec3 v2 = p2->m_position;
		b3Vec3 v3 = p3->m_position;

		b3Draw_draw->DrawTriangle(v1, v2, v3, b3Color_black);

		b3Vec3 c = (v1 + v2 + v3) / 3.0f;

		float32 s = 0.9f;

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		b3Vec3 n2 = -n1;
		b3Draw_draw->DrawSolidTriangle(n2, v3, v2, v1, b3Color_blue);
	}
}