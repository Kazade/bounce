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

#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>

b3Joint* b3Joint::Create(const b3JointDef* def)
{
	b3Joint* joint = NULL;
	switch (def->type)
	{
	case e_mouseJoint:
	{
		void* block = b3Alloc(sizeof(b3MouseJoint));
		joint = new (block) b3MouseJoint((b3MouseJointDef*)def);
		break;
	}
	case e_springJoint:
	{
		void* block = b3Alloc(sizeof(b3SpringJoint));
		joint = new (block) b3SpringJoint((b3SpringJointDef*)def);
		break;
	}
	case e_weldJoint:
	{
		void* block = b3Alloc(sizeof(b3WeldJoint));
		joint = new (block) b3WeldJoint((b3WeldJointDef*)def);
		break;
	}case e_revoluteJoint:
	{
		void* block = b3Alloc(sizeof(b3RevoluteJoint));
		joint = new (block) b3RevoluteJoint((b3RevoluteJointDef*)def);
		break;
	}
	case e_sphereJoint:
	{
		void* block = b3Alloc(sizeof(b3SphereJoint));
		joint = new (block) b3SphereJoint((b3SphereJointDef*)def);
		break;
	}
	case e_coneJoint:
	{
		void* block = b3Alloc(sizeof(b3ConeJoint));
		joint = new (block) b3ConeJoint((b3ConeJointDef*)def);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	return joint;
}

void b3Joint::Destroy(b3Joint* joint)
{
	B3_ASSERT(joint);

	b3JointType type = joint->GetType();
	switch (type)
	{
	case e_mouseJoint:
	{
		b3MouseJoint* o = (b3MouseJoint*)joint;
		o->~b3MouseJoint();
		b3Free(joint);
		break;
	}
	case e_springJoint:
	{
		b3SpringJoint* o = (b3SpringJoint*)joint;
		o->~b3SpringJoint();
		b3Free(joint);
		break;
	}
	case e_weldJoint:
	{
		b3WeldJoint* o = (b3WeldJoint*)joint;
		o->~b3WeldJoint();
		b3Free(joint);
		break;
	}
	case b3JointType::e_revoluteJoint:
	{
		b3RevoluteJoint* o = (b3RevoluteJoint*)joint;
		o->~b3RevoluteJoint();
		b3Free(joint);
		break;
	}
	case b3JointType::e_sphereJoint:
	{
		b3SphereJoint* o = (b3SphereJoint*)joint;
		o->~b3SphereJoint();
		b3Free(joint);
		break;
	}
	case b3JointType::e_coneJoint:
	{
		b3ConeJoint* o = (b3ConeJoint*)joint;
		o->~b3ConeJoint();
		b3Free(joint);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}
