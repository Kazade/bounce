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

#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/island.h>
#include <bounce/dynamics/world_listeners.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/time_step.h>

extern u32 b3_allocCalls, b3_maxAllocCalls;
extern u32 b3_convexCalls, b3_convexCacheHits;
extern u32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
extern bool b3_convexCache;

b3World::b3World() : 
	m_bodyBlocks(sizeof(b3Body))
{
	b3_allocCalls = 0;
	b3_maxAllocCalls = 0;
	
	b3_gjkCalls = 0;
	b3_gjkIters = 0;

	b3_convexCalls = 0;
	b3_convexCacheHits = 0;

	b3_convexCache = true;
	
	m_flags = e_clearForcesFlag;
	m_sleeping = false;
	m_warmStarting = true;
	m_gravity.Set(0.0f, -9.8f, 0.0f);
}

b3World::~b3World()
{
	b3Body* b = m_bodyList.m_head;
	while (b)
	{
		// Shapes and joints use b3Alloc.
		b->DestroyShapes();
		b->DestroyJoints();
		b = b->m_next;
	}
	
	b3_allocCalls = 0;
	b3_maxAllocCalls = 0;

	b3_gjkCalls = 0;
	b3_gjkIters = 0;

	b3_convexCalls = 0;
	b3_convexCacheHits = 0;
}

void b3World::SetSleeping(bool flag)
{
	m_sleeping = flag;
	if (m_sleeping == false)
	{
		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			b->SetAwake(true);
		}
	}
}

b3Body* b3World::CreateBody(const b3BodyDef& def)
{
	void* mem = m_bodyBlocks.Allocate();
	b3Body* b = new(mem) b3Body(def, this);
	m_bodyList.PushFront(b);	
	return b;
}

void b3World::DestroyBody(b3Body* b)
{
	b->DestroyShapes();
	b->DestroyJoints();
	b->DestroyContacts();
	
	m_bodyList.Remove(b);
	b->~b3Body();
	m_bodyBlocks.Free(b);
}

b3Joint* b3World::CreateJoint(const b3JointDef& def)
{
	return m_jointMan.Create(&def);
}

void b3World::DestroyJoint(b3Joint* j)
{
	m_jointMan.Destroy(j);
}

void b3World::Step(float32 dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Step");

	// Clear statistics
	b3_allocCalls = 0;
	
	b3_convexCalls = 0;
	b3_convexCacheHits = 0;

	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_gjkMaxIters = 0;

	if (m_flags & e_shapeAddedFlag)
	{
		// If new shapes were added new contacts might be created.
		m_contactMan.FindNewContacts();
		m_flags &= ~e_shapeAddedFlag;
	}

	// Update contacts. This is where some contacts might be destroyed.
	m_contactMan.UpdateContacts();

	// Integrate velocities, clear forces and torques, solve constraints, integrate positions.
	if (dt > 0.0f)
	{
		Solve(dt, velocityIterations, positionIterations);
	}
}

void b3World::Solve(float32 dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Solve");
	
	// Clear all visited flags for the depth first search.
	for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
	{
		b->m_flags &= ~b3Body::e_islandFlag;
	}

	for (b3Joint* j = m_jointMan.m_jointList.m_head; j; j = j->m_next)
	{
		j->m_flags &= ~b3Joint::e_islandFlag;
	}

	for (b3Contact* c = m_contactMan.m_contactList.m_head; c; c = c->m_next)
	{
		c->m_flags &= ~b3Contact::e_islandFlag;
	}

	u32 islandFlags = 0;
	islandFlags |= m_warmStarting * b3Island::e_warmStartBit;
	islandFlags |= m_sleeping * b3Island::e_sleepBit;

	b3Vec3 externalForce = m_gravity;

	// Create a worst case island.
	b3Island island(&m_stackAllocator, m_bodyList.m_count, m_contactMan.m_contactList.m_count, m_jointMan.m_jointList.m_count);

	// Build and simulate awake islands.
	u32 stackSize = m_bodyList.m_count;
	b3Body** stack = (b3Body**)m_stackAllocator.Allocate(stackSize * sizeof(b3Body*));
	for (b3Body* seed = m_bodyList.m_head; seed; seed = seed->m_next)
	{
		// The seed must not be on an island.
		if (seed->m_flags & b3Body::e_islandFlag)
		{
			continue;
		}

		// Bodies that are sleeping are not solved for performance.
		if (!(seed->m_flags & b3Body::e_awakeFlag))
		{
			continue;
		}

		// The seed must be dynamic or kinematic.
		if (seed->m_type == e_staticBody)
		{
			continue;
		}

		// Perform a depth first search on this body constraint graph.
		island.Clear();
		u32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b3Body::e_islandFlag;

		while (stackCount > 0)
		{
			// Add this body to the island.
			b3Body* b = stack[--stackCount];
			island.Add(b);
			
			// This body must be awake.
			b->m_flags |= b3Body::e_awakeFlag;

			// Don't propagate islands across static bodies to keep them small.
			if (b->m_type == e_staticBody)
			{
				continue;
			}

			// Search all contacts connected to this body.
			for (b3Shape* s = b->m_shapeList.m_head; s; s = s->m_next)
			{
				for (b3ContactEdge* ce = s->m_contactEdges.m_head; ce; ce = ce->m_next)
				{
					b3Contact* contact = ce->contact;

					// The contact must not be on an island.
					if (contact->m_flags & b3Contact::e_islandFlag)
					{
						continue;
					}

					// The contact must be overlapping.
					if (!(contact->m_flags & b3Contact::e_overlapFlag))
					{
						continue;
					}

					// A sensor can't respond to contacts. 
					bool sensorA = contact->GetShapeA()->m_isSensor;
					bool sensorB = contact->GetShapeB()->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Add contact to the island and mark it.
					island.Add(contact);
					contact->m_flags |= b3Contact::e_islandFlag;

					b3Body* other = ce->other->GetBody();

					// Skip adjacent vertex if it was visited.
					if (other->m_flags & b3Body::e_islandFlag)
					{
						continue;
					}

					// Add the other body to the island and propagate through it.
					B3_ASSERT(stackCount < stackSize);
					stack[stackCount++] = other;
					other->m_flags |= b3Body::e_islandFlag;
				}
			}

			// Search all joints connected to this body.
			for (b3JointEdge* je = b->m_jointEdges.m_head; je; je = je->m_next)
			{
				b3Joint* joint = je->joint;

				// The joint must not be on an island.
				if (joint->m_flags & b3Joint::e_islandFlag)
				{
					continue;
				}

				// Add joint to the island and mark it.
				island.Add(joint);
				joint->m_flags |= b3Joint::e_islandFlag;

				b3Body* other = je->other;

				// The other body must not be on an island.
				if (other->m_flags & b3Body::e_islandFlag)
				{
					continue;
				}

				// Push the other body onto the stack and mark it.
				B3_ASSERT(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b3Body::e_islandFlag;
			}
		}

		// Integrate velocities, clear forces and torques, solve constraints, integrate positions.
		island.Solve(externalForce, dt, velocityIterations, positionIterations, islandFlags);
		
		// Allow static bodies to participate in other islands.
		for (u32 i = 0; i < island.m_bodyCount; ++i)
		{
			b3Body* b = island.m_bodies[i];
			if (b->m_type == e_staticBody)
			{
				b->m_flags &= ~b3Body::e_islandFlag;
			}
		}
	}

	m_stackAllocator.Free(stack);

	{
		B3_PROFILE("Find New Pairs");

		for (b3Body* b = m_bodyList.m_head; b; b = b->m_next)
		{
			// If a body didn't participate on a island then it didn't move.
			if ((b->m_flags & b3Body::e_islandFlag) == 0)
			{
				continue;
			}

			if (b->m_type == e_staticBody)
			{
				continue;
			}

			// Update shapes for broad-phase.
			b->SynchronizeShapes();
		}

		// Notify the contacts the AABBs may have been moved.
		m_contactMan.SynchronizeShapes();

		// Find new contacts.
		m_contactMan.FindNewContacts();
	}
}

struct b3ShapeRayCastCallback
{
	float32 Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3Shape* shape = (b3Shape*)userData;
		
		// Calculate transformation from shape local space to world space.
		b3Transform xf = shape->GetBody()->GetTransform();

		b3RayCastOutput output;
		bool hit = shape->RayCast(&output, input, xf);
		if (hit)
		{
			// Ray hits shape.
			float32 fraction = output.fraction;
			float32 w1 = 1.0f - fraction;
			float32 w2 = fraction;
			
			b3Vec3 point = w1 * input.p1 + w2 * input.p2;
			b3Vec3 normal = output.normal;

			// Report the intersection to the user and get the new ray cast fraction.
			return listener->ReportShape(shape, point, normal, fraction);
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	b3RayCastListener* listener;
	const b3BroadPhase* broadPhase;
};

void b3World::RayCast(b3RayCastListener* listener, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;
	
	b3ShapeRayCastCallback callback;
	callback.listener = listener;
	callback.broadPhase = &m_contactMan.m_broadPhase;
	m_contactMan.m_broadPhase.RayCast(&callback, input);
}

struct b3RayCastSingleShapeCallback
{
	float32 Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3Shape* shape = (b3Shape*)userData;

		// Get map from shape local space to world space.
		b3Transform xf = shape->GetBody()->GetTransform();

		b3RayCastOutput output;
		bool hit = shape->RayCast(&output, input, xf);
		if (hit)
		{
			// Track minimum time of impact to require less memory.
			if (output.fraction < output0.fraction)
			{
				shape0 = shape;
				output0 = output;
			}
		}

		// Continue the search from where we stopped.
		return input.maxFraction;
	}

	b3Shape* shape0;
	b3RayCastOutput output0; 
	const b3BroadPhase* broadPhase;
};

bool b3World::RayCastSingle(b3RayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	b3RayCastSingleShapeCallback callback;
	callback.shape0 = NULL;
	callback.output0.fraction = B3_MAX_FLOAT;
	callback.broadPhase = &m_contactMan.m_broadPhase;
	
	// Perform the ray cast.
	m_contactMan.m_broadPhase.RayCast(&callback, input);

	if (callback.shape0)
	{
		// Ray hits closest shape.
		float32 fraction = callback.output0.fraction;
		b3Vec3 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
		b3Vec3 normal = callback.output0.normal;

		output->shape = callback.shape0;
		output->point = point;
		output->normal = normal;
		output->fraction = fraction;
		
		return true;
	}
	
	return false;
}

struct b3QueryAABBCallback
{
	bool Report(u32 proxyID)
	{
		b3Shape* shape = (b3Shape*)broadPhase->GetUserData(proxyID);
		return listener->ReportShape(shape);
	}

	b3QueryListener* listener;
	const b3BroadPhase* broadPhase;
};

void b3World::QueryAABB(b3QueryListener* listener, const b3AABB3& aabb) const
{
	b3QueryAABBCallback callback;
	callback.listener = listener;
	callback.broadPhase = &m_contactMan.m_broadPhase;
	m_contactMan.m_broadPhase.QueryAABB(&callback, aabb);
}
