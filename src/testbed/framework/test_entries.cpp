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

#include <testbed/tests/test.h>
#include <testbed/tests/quickhull_test.h>
#include <testbed/tests/cluster_test.h>
#include <testbed/tests/distance_test.h>
#include <testbed/tests/capsule_distance.h>
#include <testbed/tests/collide_test.h>
#include <testbed/tests/capsule_collision.h>
#include <testbed/tests/capsule_and_hull_collision.h>
#include <testbed/tests/hull_collision.h>
#include <testbed/tests/spring.h>
#include <testbed/tests/newton_cradle.h>
#include <testbed/tests/hinge_motor.h>
#include <testbed/tests/hinge_chain.h>
#include <testbed/tests/ragdoll.h>
#include <testbed/tests/quadrics.h>
#include <testbed/tests/mesh_contact_test.h>
#include <testbed/tests/sphere_stack.h>
#include <testbed/tests/capsule_stack.h>
#include <testbed/tests/box_stack.h>
#include <testbed/tests/shape_stack.h>
#include <testbed/tests/jenga.h>
#include <testbed/tests/thin.h>
#include <testbed/tests/pyramid.h>
#include <testbed/tests/pyramids.h>
#include <testbed/tests/ray_cast.h>
#include <testbed/tests/sensor_test.h>
#include <testbed/tests/character_test.h>
#include <testbed/tests/body_types.h>
#include <testbed/tests/varying_friction.h>
#include <testbed/tests/varying_restitution.h>

TestEntry g_tests[] =
{
	{ "Quickhull Test", &QuickhullTest::Create },
	{ "Cluster Test", &Cluster::Create },
	{ "Distance Test", &Distance::Create },
	{ "Capsule Distance", &CapsuleDistance::Create },
	{ "Capsule Collision", &CapsuleCollision::Create },
	{ "Capsule and Hull Collision", &CapsuleAndHull::Create },
	{ "Hull Collision", &HullAndHull::Create },
	{ "Springs", &Spring::Create },
	{ "Newton's Cradle", &NewtonCradle::Create },
	{ "Hinge Motor", &HingeMotor::Create },
	{ "Hinge Chain", &HingeChain::Create },
	{ "Ragdoll", &Ragdoll::Create },
	{ "Quadrics", &Quadric::Create },
	{ "Mesh Contact Test", &MeshContactTest::Create },
	{ "Sphere Stack", &SphereStack::Create },
	{ "Capsule Stack", &CapsuleStack::Create },
	{ "Box Stack", &BoxStack::Create },
	{ "Shape Stack", &ShapeStack::Create },
	{ "Jenga", &Jenga::Create },
	{ "Thin Plates", &Thin::Create },
	{ "Pyramid", &Pyramid::Create },
	{ "Pyramid Rows", &Pyramids::Create },
	{ "Ray Cast", &RayCast::Create },
	{ "Sensor Test", &SensorTest::Create },
	{ "Character Test", &Character::Create },
	{ "Body Types", &BodyTypes::Create },
	{ "Varying Friction", &VaryingFriction::Create },
	{ "Varying Restitution", &VaryingRestitution::Create },
	{ NULL, NULL }
};