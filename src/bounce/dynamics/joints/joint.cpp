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

#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/joints/friction_joint.h>
#include <bounce/dynamics/joints/motor_joint.h>
#include <bounce/dynamics/joints/prismatic_joint.h>
#include <bounce/dynamics/joints/wheel_joint.h>

b3Joint* b3Joint::Create(const b3JointDef* def)
{
	b3Joint* joint = nullptr;
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
	case e_frictionJoint:
	{
		void* block = b3Alloc(sizeof(b3FrictionJoint));
		joint = new (block) b3FrictionJoint((b3FrictionJointDef*)def);
		break;
	}
	case e_motorJoint:
	{
		void* block = b3Alloc(sizeof(b3MotorJoint));
		joint = new (block) b3MotorJoint((b3MotorJointDef*)def);
		break;
	}
	case e_prismaticJoint:
	{
		void* block = b3Alloc(sizeof(b3PrismaticJoint));
		joint = new (block) b3PrismaticJoint((b3PrismaticJointDef*)def);
		break;
	}
	case e_wheelJoint:
	{
		void* block = b3Alloc(sizeof(b3WheelJoint));
		joint = new (block) b3WheelJoint((b3WheelJointDef*)def);
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
	case e_revoluteJoint:
	{
		b3RevoluteJoint* o = (b3RevoluteJoint*)joint;
		o->~b3RevoluteJoint();
		b3Free(joint);
		break;
	}
	case e_sphereJoint:
	{
		b3SphereJoint* o = (b3SphereJoint*)joint;
		o->~b3SphereJoint();
		b3Free(joint);
		break;
	}
	case e_coneJoint:
	{
		b3ConeJoint* o = (b3ConeJoint*)joint;
		o->~b3ConeJoint();
		b3Free(joint);
		break;
	}
	case e_frictionJoint:
	{
		b3FrictionJoint* o = (b3FrictionJoint*)joint;
		o->~b3FrictionJoint();
		b3Free(joint);
		break;
	}
	case e_motorJoint:
	{
		b3MotorJoint* o = (b3MotorJoint*)joint;
		o->~b3MotorJoint();
		b3Free(joint);
		break;
	}
	case e_prismaticJoint:
	{
		b3PrismaticJoint* o = (b3PrismaticJoint*)joint;
		o->~b3PrismaticJoint();
		b3Free(joint);
		break;
	}
	case e_wheelJoint:
	{
		b3WheelJoint* o = (b3WheelJoint*)joint;
		o->~b3WheelJoint();
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
