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

#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/contacts/contact.h>

b3Body::b3Body(const b3BodyDef& def, b3World* world) 
{
	m_world = world;
	m_type = def.type;
	m_flags = 0;
	
	if (def.awake)
	{
		m_flags |= e_awakeFlag;
	}
	
	if (m_type == e_dynamicBody) 
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;

		if (def.fixedRotationX)
		{
			m_flags |= e_fixedRotationX;
		}

		if (def.fixedRotationY)
		{
			m_flags |= e_fixedRotationY;
		}

		if (def.fixedRotationZ)
		{
			m_flags |= e_fixedRotationZ;
		}
	}
	else 
	{
		m_mass = 0.0f;
		m_invMass = 0.0f;
	}
		
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();

	m_force.SetZero();
	m_torque.SetZero();
	
	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;

	m_sweep.localCenter.SetZero();
	m_sweep.worldCenter = def.position;
	m_sweep.orientation = def.orientation;
	m_sweep.worldCenter0 = def.position;
	m_sweep.orientation0 = def.orientation;
	m_sweep.t0 = 0.0f;

	m_xf.position = m_sweep.worldCenter;
	m_xf.rotation = b3QuatMat33(m_sweep.orientation);
	
	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;
	m_gravityScale = def.gravityScale;
	m_userData = def.userData;
	m_sleepTime = 0.0f;
}

b3Shape* b3Body::CreateShape(const b3ShapeDef& def) 
{
	// Create the shape with the definition.
	b3Shape* shape = b3Shape::Create(def);
	shape->m_body = this;
	shape->m_isSensor = def.isSensor;
	shape->m_userData = def.userData;
	shape->m_density = def.density;
	shape->m_friction = def.friction;
	shape->m_restitution = def.restitution;
	
	// Add the shape to this body shape list.
	m_shapeList.PushFront(shape);
	
	// Since a new shape was added the new mass properties of 
	// this body need to be recomputed.
	if (shape->m_density > 0.0f) 
	{
		ResetMass();
	}

	// Compute the world AABB of the new shape and assign a broad-phase proxy to it.
	b3Transform xf = m_xf;
	
	b3AABB3 aabb;
	shape->ComputeAABB(&aabb, xf);
	shape->m_broadPhaseID = m_world->m_contactMan.m_broadPhase.CreateProxy(aabb, shape);

	// Tell the world that a new shape was added so new contacts can be created.
	m_world->m_flags |= b3World::e_shapeAddedFlag;

	return shape;
}

void b3Body::DestroyContacts() 
{
	for (b3Shape* s = m_shapeList.m_head; s; s = s->m_next)
	{
		s->DestroyContacts();
	}
}

void b3Body::DestroyJoints() 
{
	b3JointEdge* je = m_jointEdges.m_head;
	while (je)
	{
		b3JointEdge* tmp = je;
		je = je->m_next;
		m_world->m_jointMan.Destroy(tmp->joint);
	}
}

void b3Body::DestroyShape(b3Shape* shape) 
{
	// Remove the shape from this body shape list.
	B3_ASSERT(shape->m_body == this);
	m_shapeList.Remove(shape);

	// Destroy any contacts associated with the shape.
	shape->DestroyContacts();
	
	// Destroy the broad-phase proxy associated with the shape.
	m_world->m_contactMan.m_broadPhase.DestroyProxy(shape->m_broadPhaseID);
	
	// Destroy the shape.
	b3Shape::Destroy(shape);

	// Recalculate the new inertial properties of this body.
	ResetMass();
}

void b3Body::DestroyShapes() 
{
	// Destroy all shapes that belong to this body.
	b3Shape* s = m_shapeList.m_head;
	while (s) 
	{
		b3Shape* s0 = s;
		s = s->m_next;
		s0->DestroyContacts();
		m_world->m_contactMan.m_broadPhase.DestroyProxy(s0->m_broadPhaseID);
		m_shapeList.Remove(s0);
		b3Shape::Destroy(s0);
	}
}

void b3Body::SynchronizeTransform()
{
	m_xf = m_sweep.GetTransform(1.0f);
}

void b3Body::SynchronizeShapes() 
{
	b3Transform xf1 = m_sweep.GetTransform(0.0f);

	b3Transform xf2 = m_xf;
	
	b3Vec3 displacement = xf2.position - xf1.position;

	// Update all shape AABBs.
	b3BroadPhase* broadPhase = &m_world->m_contactMan.m_broadPhase;
	for (b3Shape* s = m_shapeList.m_head; s; s = s->m_next)
	{
		// Compute an AABB that encloses the swept shape AABB.
		b3AABB3 aabb1, aabb2;
		s->ComputeAABB(&aabb1, xf1);
		s->ComputeAABB(&aabb2, xf2);
		
		b3AABB3 aabb = b3Combine(aabb1, aabb2);

		broadPhase->MoveProxy(s->m_broadPhaseID, aabb, displacement);
	}
}

bool b3Body::ShouldCollide(const b3Body* other) const
{
	if (m_type != e_dynamicBody && other->m_type != e_dynamicBody)
	{
		// At least one body should be kinematic or dynamic.
		return false;
	}

	// Check if there are joints that connects
	// this body with the other body and if the joint was configured 
	// to let not their collision occur.
	for (b3JointEdge* je = m_jointEdges.m_head; je; je = je->m_next)
	{
		b3Joint* j = je->joint;
		if (je->other == other)
		{
			if (j->m_collideLinked == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b3Body::ResetMass() 
{
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();
	m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (m_type == e_staticBody || m_type == e_kinematicBody)
	{
		m_sweep.worldCenter0 = m_xf.position;
		m_sweep.worldCenter = m_xf.position;
		m_sweep.orientation0 = m_sweep.orientation;
		return;
	}

	// Accumulate the mass about the body origin of all shapes.
	b3Vec3 localCenter;
	localCenter.SetZero();
	for (b3Shape* s = m_shapeList.m_head; s; s = s->m_next)
	{
		if (s->m_density == 0.0f) 
		{
			continue;
		}
	
		b3MassData massData;
		s->ComputeMass(&massData, s->m_density);
		
		localCenter += massData.mass * massData.center;
		m_mass += massData.mass;
		m_I += massData.I;
	}

	if (m_mass > 0.0f) 
	{
		// Compute local center of mass.
		m_invMass = 1.0f / m_mass;
		localCenter *= m_invMass;

		// Shift inertia about the body origin into the body local center of mass.
		m_I = m_I - m_mass * b3Steiner(localCenter);
		
		B3_ASSERT(m_I.x.x > 0.0f);
		B3_ASSERT(m_I.y.y > 0.0f);
		B3_ASSERT(m_I.z.z > 0.0f);

		// Compute inverse inertia about the body local center of mass.
		m_invI = b3Inverse(m_I);

		// Align the inverse inertia with the world frame of the body.
		m_worldInvI = b3RotateToFrame(m_invI, m_xf.rotation);

		// Fix rotation.
		if (m_flags & e_fixedRotationX)
		{
			m_invI.y.y = 0.0f;
			m_invI.z.y = 0.0f;
			m_invI.y.z = 0.0f;
			m_invI.z.z = 0.0f;

			m_worldInvI.y.y = 0.0f;
			m_worldInvI.z.y = 0.0f;
			m_worldInvI.y.z = 0.0f;
			m_worldInvI.z.z = 0.0f;
		}

		if (m_flags & e_fixedRotationY)
		{
			m_invI.x.x = 0.0f;
			m_invI.x.z = 0.0f;
			m_invI.z.x = 0.0f;
			m_invI.z.z = 0.0f;

			m_worldInvI.x.x = 0.0f;
			m_worldInvI.x.z = 0.0f;
			m_worldInvI.z.x = 0.0f;
			m_worldInvI.z.z = 0.0f;
		}

		if (m_flags & e_fixedRotationZ)
		{
			m_invI.x.x = 0.0f;
			m_invI.x.y = 0.0f;
			m_invI.y.x = 0.0f;
			m_invI.y.y = 0.0f;

			m_worldInvI.x.x = 0.0f;
			m_worldInvI.x.y = 0.0f;
			m_worldInvI.y.x = 0.0f;
			m_worldInvI.y.y = 0.0f;
		}
	}
	else 
	{
		// Force all dynamic bodies to have positive mass.
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}

	// Move center of mass.
	b3Vec3 oldCenter = m_sweep.worldCenter;
	m_sweep.localCenter = localCenter;
	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.worldCenter0 = m_sweep.worldCenter;

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.worldCenter - oldCenter);
}

void b3Body::GetMassData(b3MassData* data) const
{
	data->mass = m_mass;
	data->I = m_I;
	data->center = m_sweep.localCenter;
}

void b3Body::SetMassData(const b3MassData* massData)
{
	if (m_type != e_dynamicBody)
	{
		return;
	}

	m_invMass = 0.0f;
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();

	m_mass = massData->mass;
	if (m_mass > 0.0f)
	{
		m_invMass = 1.0f / m_mass;
		m_I = massData->I - m_mass * b3Steiner(massData->center);
		
		B3_ASSERT(m_I.x.x > 0.0f);
		B3_ASSERT(m_I.y.y > 0.0f);
		B3_ASSERT(m_I.z.z > 0.0f);

		m_invI = b3Inverse(m_I);
		m_worldInvI = b3RotateToFrame(m_invI, m_xf.rotation);

		if (m_flags & e_fixedRotationX)
		{
			m_invI.y.y = 0.0f;
			m_invI.z.y = 0.0f;
			m_invI.y.z = 0.0f;
			m_invI.z.z = 0.0f;

			m_worldInvI.y.y = 0.0f;
			m_worldInvI.z.y = 0.0f;
			m_worldInvI.y.z = 0.0f;
			m_worldInvI.z.z = 0.0f;
		}

		if (m_flags & e_fixedRotationY)
		{
			m_invI.x.x = 0.0f;
			m_invI.x.z = 0.0f;
			m_invI.z.x = 0.0f;
			m_invI.z.z = 0.0f;

			m_worldInvI.x.x = 0.0f;
			m_worldInvI.x.z = 0.0f;
			m_worldInvI.z.x = 0.0f;
			m_worldInvI.z.z = 0.0f;
		}

		if (m_flags & e_fixedRotationZ)
		{
			m_invI.x.x = 0.0f;
			m_invI.x.y = 0.0f;
			m_invI.y.x = 0.0f;
			m_invI.y.y = 0.0f;

			m_worldInvI.x.x = 0.0f;
			m_worldInvI.x.y = 0.0f;
			m_worldInvI.y.x = 0.0f;
			m_worldInvI.y.y = 0.0f;
		}
	}
	else
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}

	// Move center of mass.
	b3Vec3 oldCenter = m_sweep.worldCenter;
	m_sweep.localCenter = massData->center;
	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.worldCenter0 = m_sweep.worldCenter;

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.worldCenter - oldCenter);
}

void b3Body::SetType(b3BodyType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMass();

	m_force.SetZero();
	m_torque.SetZero();

	if (m_type == e_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();
		m_sweep.worldCenter0 = m_sweep.worldCenter;
		m_sweep.orientation0 = m_sweep.orientation;
		SynchronizeShapes();
	}

	SetAwake(true);

	DestroyContacts();

	// Move the shape proxies so new contacts can be created.
	b3BroadPhase* phase = &m_world->m_contactMan.m_broadPhase;
	for (b3Shape* s = m_shapeList.m_head; s; s = s->m_next)
	{
		phase->BufferMove(s->m_broadPhaseID);
	}
}

void b3Body::Dump() const
{
	u32 bodyIndex = m_islandID;

	b3Log("		{\n");
	b3Log("		b3BodyDef bd;\n");
	b3Log("		bd.type = (b3BodyType) %d;\n", m_type);
	b3Log("		bd.position.Set(%f, %f, %f);\n", m_sweep.worldCenter.x, m_sweep.worldCenter.y, m_sweep.worldCenter.z);
	b3Log("		bd.orientation.Set(%f, %f, %f, %f);\n", m_sweep.orientation.x, m_sweep.orientation.y, m_sweep.orientation.z, m_sweep.orientation.w);
	b3Log("		bd.linearVelocity.Set(%f, %f, %f);\n", m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z);
	b3Log("		bd.angularVelocity.Set(%f, %f, %f);\n", m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
	b3Log("		bd.gravityScale = %f;\n", m_gravityScale);
	b3Log("		bd.awake = %d;\n", m_flags & e_awakeFlag);
	b3Log("		\n");
	b3Log("		bodies[%d] = world.CreateBody(bd);\n");
	b3Log("		\n");
	for (b3Shape* s = m_shapeList.m_head; s; s = s->m_next)
	{
		b3Log("		{\n");
		s->Dump(bodyIndex);
		b3Log("		}\n");
	}
	b3Log("		}\n");
}
