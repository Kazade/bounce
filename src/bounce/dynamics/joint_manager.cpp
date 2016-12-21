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

#include <bounce/dynamics/joint_manager.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/body.h>

b3JointManager::b3JointManager() 
{
}

b3Joint* b3JointManager::Create(const b3JointDef* def) 
{
	b3Body* bodyA = def->bodyA;
	b3Body* bodyB = def->bodyB;

	// There must not be one joint linking the same bodies.
	B3_ASSERT(bodyA != bodyB);
	if (bodyA == bodyB)
	{
		return NULL;
	}

	// Allocate the new joint.
	b3Joint* j = b3Joint::Create(def);
	j->m_flags = 0;
	j->m_collideLinked = def->collideLinked;
	j->m_userData = def->userData;

	// Add the joint to body A's joint edge list
	j->m_pair.bodyA = bodyA;
	j->m_pair.edgeA.other = bodyB;
	j->m_pair.edgeA.joint = j;
	bodyA->m_jointEdges.PushFront(&j->m_pair.edgeA);

	// Add the joint to body B's joint edge list
	j->m_pair.bodyB = bodyB;
	j->m_pair.edgeB.other = bodyA;
	j->m_pair.edgeB.joint = j;
	bodyB->m_jointEdges.PushFront(&j->m_pair.edgeB);

	// Add the joint to the world joint list
	m_jointList.PushFront(j);

	// Creating a joint doesn't awake the bodies.

	return j;
}

void b3JointManager::Destroy(b3Joint* j) 
{
	b3Body* bodyA = j->GetBodyA();
	b3Body* bodyB = j->GetBodyB();

	// Remove the joint from body A's joint list.
	bodyA->m_jointEdges.Remove(&j->m_pair.edgeA);

	// Remove the joint from body B's joint list.
	bodyB->m_jointEdges.Remove(&j->m_pair.edgeB);

	// Remove the joint from the world joint list.
	m_jointList.Remove(j);
	
	// Destroy the joint.
	b3Joint::Destroy(j);
}
