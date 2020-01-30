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
#include <bounce/cloth/cloth_particle.h>
#include <bounce/cloth/shapes/cloth_sphere_shape.h>
#include <bounce/cloth/shapes/cloth_capsule_shape.h>
#include <bounce/cloth/shapes/cloth_triangle_shape.h>
#include <bounce/cloth/shapes/cloth_world_shape.h>
#include <bounce/cloth/cloth_solver.h>
#include <bounce/cloth/cloth_time_step.h>
#include <bounce/cloth/forces/stretch_force.h>
#include <bounce/cloth/forces/shear_force.h>
#include <bounce/cloth/forces/spring_force.h>
#include <bounce/cloth/forces/element_force.h>
#include <bounce/common/draw.h>

b3Cloth::b3Cloth() :
	m_particleBlocks(sizeof(b3ClothParticle)),
	m_sphereShapeBlocks(sizeof(b3ClothSphereShape)),
	m_capsuleShapeBlocks(sizeof(b3ClothCapsuleShape)),
	m_triangleShapeBlocks(sizeof(b3ClothTriangleShape)),
	m_worldShapeBlocks(sizeof(b3ClothWorldShape))
{
	m_contactManager.m_cloth = this;
	m_gravity.SetZero();
	m_enableSelfCollision = false;
	m_inv_dt0 = scalar(0);
	m_mesh = nullptr;
	m_particles = nullptr;
	m_spheres = nullptr;
	m_triangles = nullptr;
}

b3Cloth::b3Cloth(const b3ClothDef& def) :
	m_particleBlocks(sizeof(b3ClothParticle)),
	m_sphereShapeBlocks(sizeof(b3ClothSphereShape)),
	m_capsuleShapeBlocks(sizeof(b3ClothCapsuleShape)),
	m_triangleShapeBlocks(sizeof(b3ClothTriangleShape)),
	m_worldShapeBlocks(sizeof(b3ClothWorldShape))
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > scalar(0));

	m_contactManager.m_cloth = this;
	m_gravity.SetZero();
	m_enableSelfCollision = false;
	m_inv_dt0 = scalar(0);
	m_mesh = def.mesh;

	const b3ClothMesh* m = def.mesh;

	// Create particles and spheres
	m_particles = (b3ClothParticle * *)b3Alloc(m->vertexCount * sizeof(b3ClothParticle*));
	m_spheres = (b3ClothSphereShape * *)b3Alloc(m->vertexCount * sizeof(b3ClothSphereShape*));
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3ClothParticleDef pd;
		pd.type = e_dynamicClothParticle;
		pd.position = m->vertices[i];
		pd.meshIndex = i;

		m_particles[i] = nullptr;
		b3ClothParticle* p = CreateParticle(pd);

		b3ClothSphereShapeDef sd;
		sd.p = p;
		sd.radius = def.thickness;
		sd.friction = def.friction;
		sd.meshIndex = i;

		m_spheres[i] = nullptr;
		b3ClothSphereShape* s = CreateSphereShape(sd);
	}

	// Create triangles and capsules
	m_triangles = (b3ClothTriangleShape * *)b3Alloc(m->triangleCount * sizeof(b3ClothTriangleShape*));
	for (u32 i = 0; i < m->triangleCount; ++i)
	{
		b3ClothMeshTriangle* meshTriangle = m->triangles + i;

		u32 v1 = meshTriangle->v1;
		u32 v2 = meshTriangle->v2;
		u32 v3 = meshTriangle->v3;

		b3ClothParticle* p1 = m_particles[v1];
		b3ClothParticle* p2 = m_particles[v2];
		b3ClothParticle* p3 = m_particles[v3];
		
		b3ClothTriangleShapeDef td;
		td.p1 = p1;
		td.p2 = p2;
		td.p3 = p3;
		td.v1 = m->vertices[v1];
		td.v2 = m->vertices[v2];
		td.v3 = m->vertices[v3];
		td.radius = def.thickness;
		td.friction = def.friction;
		td.meshIndex = i;

		m_triangles[i] = nullptr;
		b3ClothTriangleShape* ts = CreateTriangleShape(td);

		{
			b3ClothCapsuleShapeDef sd;
			sd.p1 = p1;
			sd.p2 = p2;
			sd.radius = def.thickness;
			sd.friction = def.friction;

			CreateCapsuleShape(sd);
		}

		{
			b3ClothCapsuleShapeDef sd;
			sd.p1 = p2;
			sd.p2 = p3;
			sd.radius = def.thickness;
			sd.friction = def.friction;

			CreateCapsuleShape(sd);
		}

		{
			b3ClothCapsuleShapeDef sd;
			sd.p1 = p3;
			sd.p2 = p1;
			sd.radius = def.thickness;
			sd.friction = def.friction;

			CreateCapsuleShape(sd);
		}
	}

	if (def.streching > scalar(0))
	{
		// Streching
		for (u32 i = 0; i < m->triangleCount; ++i)
		{
			b3ClothMeshTriangle* t = m->triangles + i;

			u32 v1 = t->v1;
			u32 v2 = t->v2;
			u32 v3 = t->v3;

			b3ClothParticle* p1 = m_particles[v1];
			b3ClothParticle* p2 = m_particles[v2];
			b3ClothParticle* p3 = m_particles[v3];

			b3Vec3 x1 = m->vertices[v1];
			b3Vec3 x2 = m->vertices[v2];
			b3Vec3 x3 = m->vertices[v3];

			b3StretchForceDef fd;
			fd.Initialize(x1, x2, x3);
			
			fd.p1 = p1;
			fd.p2 = p2;
			fd.p3 = p3;
			fd.stretching_u = def.streching;
			fd.damping_u = def.strechDamping;
			fd.b_u = scalar(1);
			fd.stretching_v = def.streching;
			fd.damping_v = def.strechDamping;
			fd.b_v = scalar(1);

			CreateForce(fd);
		}
	}

	if (def.shearing > scalar(0))
	{
		// Shearing
		for (u32 i = 0; i < m->shearingLineCount; ++i)
		{
			b3ClothMeshShearingLine* line = m->shearingLines + i;

			b3ClothParticle* p1 = m_particles[line->v1];
			b3ClothParticle* p2 = m_particles[line->v2];

			b3SpringForceDef fd;
			fd.Initialize(p1, p2, def.shearing, def.shearDamping);

			CreateForce(fd);
		}
	}

	if (def.bending > scalar(0))
	{
		// Bending
		for (u32 i = 0; i < m->bendingLineCount; ++i)
		{
			b3ClothMeshBendingLine* line = m->bendingLines + i;

			b3ClothParticle* p1 = m_particles[line->v1];
			b3ClothParticle* p2 = m_particles[line->v2];

			b3SpringForceDef fd;
			fd.Initialize(p1, p2, def.bending, def.bendDamping);

			CreateForce(fd);
		}
	}

	if (def.sewing > scalar(0))
	{
		// Sewing
		for (u32 i = 0; i < m->sewingLineCount; ++i)
		{
			b3ClothMeshSewingLine* line = m->sewingLines + i;

			b3ClothParticle* p1 = m_particles[line->v1];
			b3ClothParticle* p2 = m_particles[line->v2];

			b3SpringForceDef fd;
			fd.Initialize(p1, p2, def.sewing, def.sewDamping);

			CreateForce(fd);
		}
	}
}

b3Cloth::~b3Cloth()
{
	b3Force* f = m_forceList.m_head;
	while (f)
	{
		b3Force* boom = f;
		f = f->m_next;
		b3Force::Destroy(boom);
	}

	b3Free(m_particles);
	b3Free(m_spheres);
	b3Free(m_triangles);
}

b3ClothParticle* b3Cloth::CreateParticle(const b3ClothParticleDef& def)
{
	void* mem = m_particleBlocks.Allocate();
	b3ClothParticle* p = new(mem) b3ClothParticle(def, this);
	
	if (p->m_meshIndex != B3_MAX_U32)
	{
		B3_ASSERT(m_particles[p->m_meshIndex] == nullptr);
		m_particles[p->m_meshIndex] = p;
	}

	m_particleList.PushFront(p);

	return p;
}

void b3Cloth::DestroyParticle(b3ClothParticle* particle)
{
	if (particle->m_meshIndex != B3_MAX_U32)
	{
		m_particles[particle->m_meshIndex] = nullptr;
	}

	// Destroy shapes
	particle->DestroySpheres();
	particle->DestroyCapsules();
	particle->DestroyTriangles();

	// Destroy forces
	particle->DestroyForces();

	// Destroy contacts
	particle->DestroyContacts();

	m_particleList.Remove(particle);
	particle->~b3ClothParticle();
	m_particleBlocks.Free(particle);
}

b3ClothSphereShape* b3Cloth::CreateSphereShape(const b3ClothSphereShapeDef& def)
{
	// Check if the shape exists.
	for (b3ClothSphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == def.p)
		{
			return s;
		}
	}

	void* mem = m_sphereShapeBlocks.Allocate();
	b3ClothSphereShape* s = new (mem)b3ClothSphereShape(def, this);

	s->m_radius = def.radius;
	s->m_friction = def.friction;
	s->m_density = def.density;
	s->m_meshIndex = def.meshIndex;

	if (s->m_meshIndex != B3_MAX_U32)
	{
		B3_ASSERT(m_spheres[s->m_meshIndex] == nullptr);
		m_spheres[s->m_meshIndex] = s;
	}

	b3AABB aabb = s->ComputeAABB();
	s->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, s);

	m_sphereShapeList.PushFront(s);

	return s;
}

void b3Cloth::DestroySphereShape(b3ClothSphereShape* shape)
{
	if (shape->m_meshIndex != B3_MAX_U32)
	{
		m_spheres[shape->m_meshIndex] = nullptr;
	}

	// Destroy contacts
	shape->DestroyContacts();

	// Remove shape from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	m_sphereShapeList.Remove(shape);
	shape->~b3ClothSphereShape();
	m_sphereShapeBlocks.Free(shape);
}

b3ClothCapsuleShape* b3Cloth::CreateCapsuleShape(const b3ClothCapsuleShapeDef& def)
{
	// Check if the shape exists.
	for (b3ClothCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == def.p1 && c->m_p2 == def.p2)
		{
			return c;
		}

		if (c->m_p1 == def.p2 && c->m_p2 == def.p1)
		{
			return c;
		}
	}

	void* mem = m_capsuleShapeBlocks.Allocate();
	b3ClothCapsuleShape* c = new (mem)b3ClothCapsuleShape(def, this);

	c->m_radius = def.radius;
	c->m_friction = def.friction;
	c->m_density = def.density;
	c->m_meshIndex = def.meshIndex;

	b3AABB aabb = c->ComputeAABB();
	c->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, c);

	m_capsuleShapeList.PushFront(c);

	return c;
}

void b3Cloth::DestroyCapsuleShape(b3ClothCapsuleShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remove shape from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	m_capsuleShapeList.Remove(shape);
	shape->~b3ClothCapsuleShape();
	m_capsuleShapeBlocks.Free(shape);
}

b3ClothTriangleShape* b3Cloth::CreateTriangleShape(const b3ClothTriangleShapeDef& def)
{
	b3ClothParticle* p1 = def.p1;
	b3ClothParticle* p2 = def.p2;
	b3ClothParticle* p3 = def.p3;

	for (b3ClothTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		bool hasP1 = t->m_p1 == p1 || t->m_p2 == p1 || t->m_p3 == p1;
		bool hasP2 = t->m_p1 == p2 || t->m_p2 == p2 || t->m_p3 == p2;
		bool hasP3 = t->m_p1 == p3 || t->m_p2 == p3 || t->m_p3 == p3;

		if (hasP1 && hasP2 && hasP3)
		{
			return t;
		}
	}

	void* mem = m_triangleShapeBlocks.Allocate();
	b3ClothTriangleShape* t = new (mem)b3ClothTriangleShape(def, this);
	
	t->m_radius = def.radius;
	t->m_friction = def.friction;
	t->m_density = def.density;
	t->m_meshIndex = def.meshIndex;

	if (t->m_meshIndex != B3_MAX_U32)
	{
		B3_ASSERT(m_triangles[t->m_meshIndex] == nullptr);
		m_triangles[t->m_meshIndex] = t;
	}

	b3AABB aabb = t->ComputeAABB();
	t->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, t);

	m_triangleShapeList.PushFront(t);

	// Reset the cloth mass
	ResetMass();

	return t;
}

void b3Cloth::DestroyTriangleShape(b3ClothTriangleShape* shape)
{
	if (shape->m_meshIndex != B3_MAX_U32)
	{
		m_triangles[shape->m_meshIndex] = nullptr;
	}

	// Destroy contacts
	shape->DestroyContacts();

	// Remove shape from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	m_triangleShapeList.Remove(shape);
	shape->~b3ClothTriangleShape();
	m_triangleShapeBlocks.Free(shape);

	// Reset the cloth mass
	ResetMass();
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

b3ClothWorldShape* b3Cloth::CreateWorldShape(const b3ClothWorldShapeDef& def)
{
	void* mem = m_worldShapeBlocks.Allocate();
	b3ClothWorldShape* s = new (mem)b3ClothWorldShape(def, this);

	b3AABB aabb = s->ComputeAABB();
	s->m_broadPhaseId = m_contactManager.m_broadPhase.CreateProxy(aabb, s);

	m_worldShapeList.PushFront(s);

	return s;
}

void b3Cloth::DestroyWorldShape(b3ClothWorldShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remote from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_broadPhaseId);

	m_worldShapeList.Remove(shape);
}

b3ClothParticle* b3Cloth::GetParticle(u32 index)
{
	B3_ASSERT(index < m_mesh->vertexCount);
	return m_particles[index];
}

b3ClothSphereShape* b3Cloth::GetSphere(u32 index)
{
	B3_ASSERT(index < m_mesh->vertexCount);
	return m_spheres[index];
}

b3ClothTriangleShape* b3Cloth::GetTriangle(u32 index)
{
	B3_ASSERT(index < m_mesh->triangleCount);
	return m_triangles[index];
}

void b3Cloth::EnableSelfCollision(bool flag)
{
	if (m_enableSelfCollision == true && flag == false)
	{
		{
			// Destroy triangle contacts
			b3ClothSphereAndTriangleContact* c = m_contactManager.m_sphereAndTriangleContactList.m_head;
			while (c)
			{
				b3ClothSphereAndTriangleContact* boom = c;
				c = c->m_next;
				m_contactManager.Destroy(boom);
			}
		}
	
		{
			// Destroy capsule contacts
			b3ClothCapsuleAndCapsuleContact* c = m_contactManager.m_capsuleAndCapsuleContactList.m_head;
			while (c)
			{
				b3ClothCapsuleAndCapsuleContact* boom = c;
				c = c->m_next;
				m_contactManager.Destroy(boom);
			}
		}
	}

	m_enableSelfCollision = flag;
}

scalar b3Cloth::GetEnergy() const
{
	scalar E = scalar(0);
	for (b3ClothParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		E += p->m_mass * b3Dot(p->m_velocity, p->m_velocity);
	}
	return scalar(0.5) * E;
}

void b3Cloth::ResetMass()
{
	for (b3ClothTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		t->m_p1->m_mass = scalar(0);
		t->m_p2->m_mass = scalar(0);
		t->m_p3->m_mass = scalar(0);
	}

	const scalar inv3 = scalar(1) / scalar(3);

	for (b3ClothTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3ClothParticle* p1 = t->m_p1;
		b3ClothParticle* p2 = t->m_p2;
		b3ClothParticle* p3 = t->m_p3;

		scalar mass = t->m_density * t->m_area;
		
		p1->m_mass += inv3 * mass;
		p2->m_mass += inv3 * mass;
		p3->m_mass += inv3 * mass;
	}

	// Invert
	for (b3ClothParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		// Static and kinematic particles have zero mass.
		if (p->m_type == e_staticClothParticle || p->m_type == e_kinematicClothParticle)
		{
			p->m_mass = scalar(0);
			p->m_invMass = scalar(0);
			continue;
		}

		if (p->m_mass > scalar(0))
		{
			p->m_invMass = scalar(1) / p->m_mass;
		}
		else
		{
			// Force all dynamic particles to have non-zero mass.
			p->m_mass = scalar(1);
			p->m_invMass = scalar(1);
		}
	}
}

struct b3ClothRayCastSingleCallback
{
	scalar Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3ClothShape* shape = (b3ClothShape*)userData;

		if (shape->GetType() != e_clothTriangleShape)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3ClothTriangleShape* triangleShape = (b3ClothTriangleShape*)shape;

		b3Vec3 v1 = triangleShape->GetParticle1()->GetPosition();
		b3Vec3 v2 = triangleShape->GetParticle2()->GetPosition();
		b3Vec3 v3 = triangleShape->GetParticle3()->GetPosition();

		b3RayCastOutput subOutput;
		if (b3RayCast(&subOutput, &input, v1, v2, v3))
		{
			// Ray hits triangle.
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = triangleShape;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	const b3Cloth* cloth;
	const b3BroadPhase* broadPhase;
	b3ClothTriangleShape* triangle0;
	b3RayCastOutput output0;
};

bool b3Cloth::RayCastSingle(b3ClothRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	b3ClothRayCastSingleCallback callback;
	callback.cloth = this;
	callback.broadPhase = &m_contactManager.m_broadPhase;
	callback.triangle0 = nullptr;
	callback.output0.fraction = B3_MAX_SCALAR;

	m_contactManager.m_broadPhase.RayCast(&callback, input);

	if (callback.triangle0 != nullptr)
	{
		output->triangle = callback.triangle0;
		output->fraction = callback.output0.fraction;
		output->normal = callback.output0.normal;

		return true;
	}

	return false;
}

void b3Cloth::Solve(const b3ClothTimeStep& step)
{
	B3_PROFILE("Cloth Solve");

	// Solve
	b3ClothSolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.shapeContactCapacity = m_contactManager.m_sphereAndShapeContactList.m_count;
	solverDef.triangleContactCapacity = m_contactManager.m_sphereAndTriangleContactList.m_count;
	solverDef.capsuleContactCapacity = m_contactManager.m_capsuleAndCapsuleContactList.m_count;
	
	b3ClothSolver solver(solverDef);

	for (b3ClothParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3ClothSphereAndTriangleContact* c = m_contactManager.m_sphereAndTriangleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	for (b3ClothCapsuleAndCapsuleContact* c = m_contactManager.m_capsuleAndCapsuleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}
	
	for (b3ClothSphereAndShapeContact* c = m_contactManager.m_sphereAndShapeContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	// Solve	
	solver.Solve(step, m_gravity);
}

void b3Cloth::Step(scalar dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Cloth Step");

	// Update contacts
	m_contactManager.UpdateContacts();

	// Time step parameters
	b3ClothTimeStep step;
	step.dt = dt;
	step.velocityIterations = velocityIterations;
	step.positionIterations = positionIterations;
	step.inv_dt = dt > scalar(0) ? scalar(1) / dt : scalar(0);
	step.dt_ratio = m_inv_dt0 * dt;

	// Integrate state, solve constraints. 
	if (step.dt > scalar(0))
	{
		Solve(step);
	}

	if (step.dt > scalar(0))
	{
		m_inv_dt0 = step.inv_dt;
	}

	// Clear external applied forces and translations
	for (b3ClothParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}
	
	// Synchronize sphere shapes
	for (b3ClothSphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		b3ClothParticle* p = s->m_p;

		// Synchronize unconditionally  because all particles can be translated.
		b3Vec3 displacement = dt * p->m_velocity;
		
		s->Synchronize(displacement);
	}

	// Synchronize capsule shapes
	for (b3ClothCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		b3ClothParticle* p1 = c->m_p1;
		b3ClothParticle* p2 = c->m_p2;

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;

		// Center velocity
		b3Vec3 velocity = scalar(0.5) * (v1 + v2);

		b3Vec3 displacement = dt * velocity;

		c->Synchronize(displacement);
	}
	
	// Synchronize triangle shapes
	for (b3ClothTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3ClothParticle* p1 = t->m_p1;
		b3ClothParticle* p2 = t->m_p2;
		b3ClothParticle* p3 = t->m_p3;

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;
		b3Vec3 v3 = p3->m_velocity;

		// Center velocity
		b3Vec3 velocity = (v1 + v2 + v3) / scalar(3);

		b3Vec3 displacement = dt * velocity;

		t->Synchronize(displacement);
	}

	// Synchronize world shapes
	for (b3ClothWorldShape* s = m_worldShapeList.m_head; s; s = s->m_next)
	{
		s->Synchronize(b3Vec3_zero);
	}

	// Find new contacts
	m_contactManager.FindNewContacts();
}

void b3Cloth::Draw() const
{
	for (b3ClothParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_type == e_staticClothParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0, b3Color_white);
		}

		if (p->m_type == e_kinematicClothParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0, b3Color_blue);
		}

		if (p->m_type == e_dynamicClothParticle)
		{
			b3Draw_draw->DrawPoint(p->m_position, 4.0, b3Color_green);
		}
	}

	for (b3ClothCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		b3ClothParticle* p1 = c->m_p1;
		b3ClothParticle* p2 = c->m_p2;

		b3Draw_draw->DrawSegment(p1->m_position, p2->m_position, b3Color_black);
	}

	for (b3ClothTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3ClothParticle* p1 = t->m_p1;
		b3ClothParticle* p2 = t->m_p2;
		b3ClothParticle* p3 = t->m_p3;

		b3Vec3 v1 = p1->m_position;
		b3Vec3 v2 = p2->m_position;
		b3Vec3 v3 = p3->m_position;
		
		b3Vec3 c = (v1 + v2 + v3) / scalar(3);

		scalar s(0.9);

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();

		// Solid radius
		const scalar rs(0.05);
		
		// Frame radius plus a small tolerance to prevent z-fighting
		const scalar rf = rs + scalar(0.005);

		b3Color frontSolidColor(scalar(0), scalar(0), scalar(1));
		b3Color frontFrameColor(scalar(0), scalar(0), scalar(0.5));

		b3Color backSolidColor(scalar(0.5), scalar(0.5), scalar(0.5));
		b3Color backFrameColor(scalar(0.25), scalar(0.25), scalar(0.25));

		{
			b3Vec3 x1 = v1 + rf * n;
			b3Vec3 x2 = v2 + rf * n;
			b3Vec3 x3 = v3 + rf * n;

			b3Draw_draw->DrawTriangle(x1, x2, x3, frontFrameColor);
		}
		
		{
			b3Vec3 x1 = v1 - rf * n;
			b3Vec3 x2 = v2 - rf * n;
			b3Vec3 x3 = v3 - rf * n;

			b3Draw_draw->DrawTriangle(x1, x2, x3, backFrameColor);
		}

		{
			b3Vec3 x1 = v1 + rs * n;
			b3Vec3 x2 = v2 + rs * n;
			b3Vec3 x3 = v3 + rs * n;

			b3Draw_draw->DrawSolidTriangle(n, x1, x2, x3, frontSolidColor);
		}
		
		{
			b3Vec3 x1 = v1 - rs * n;
			b3Vec3 x2 = v2 - rs * n;
			b3Vec3 x3 = v3 - rs * n;

			b3Draw_draw->DrawSolidTriangle(-n, x3, x2, x1, backSolidColor);
		}
	}
}