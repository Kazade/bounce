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
#include <testbed/tests/collide_test.h>
#include <testbed/tests/capsule_collision.h>
#include <testbed/tests/capsule_and_hull_collision_1.h>
#include <testbed/tests/capsule_and_hull_collision_2.h>
#include <testbed/tests/hull_collision.h>
#include <testbed/tests/hull_collision_2.h>
#include <testbed/tests/linear_motion.h>
#include <testbed/tests/angular_motion.h>
#include <testbed/tests/initial_overlap.h>
#include <testbed/tests/capsule_and_hull_contact_1.h>
#include <testbed/tests/quadric_shapes.h>
#include <testbed/tests/multiple_shapes.h>
#include <testbed/tests/gyro_test.h>
#include <testbed/tests/spring.h>
#include <testbed/tests/weld_test.h>
#include <testbed/tests/cone_test.h>
#include <testbed/tests/hinge_motor.h>
#include <testbed/tests/hinge_chain.h>
#include <testbed/tests/newton_cradle.h>
#include <testbed/tests/ragdoll.h>
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
#include <testbed/tests/cloth_test.h>
#include <testbed/tests/tumbler.h>

TestEntry g_tests[] =
{
	{ "Quickhull Test", &QuickhullTest::Create },
	{ "Cluster Test", &Cluster::Create },
	{ "Distance Test", &Distance::Create },
	{ "Capsule Collision", &CapsuleCollision::Create },
	{ "Capsule and Hull Collision (1)", &CapsuleAndHullCollision1::Create },
	{ "Capsule and Hull Collision (2)", &CapsuleAndHullCollision2::Create },
	{ "Hull Collision (1)", &HullAndHull::Create },
	{ "Hull Collision (2)", &HullAndHull2::Create },
	{ "Capsule and Hull Contact (1)", &CapsuleAndHullContact1::Create },
	{ "Linear Motion", &LinearMotion::Create },
	{ "Angular Motion", &AngularMotion::Create },
	{ "Multiple Shapes", &MultipleShapes::Create },
	{ "Quadric Shapes", &QuadricShapes::Create },
	{ "Thin Boxes", &Thin::Create },
	{ "Gyroscopic Test", &GyroTest::Create },
	{ "Springs", &Spring::Create },
	{ "Weld Test", &WeldTest::Create },
	{ "Cone Test", &ConeTest::Create },
	{ "Hinge Motor", &HingeMotor::Create },
	{ "Hinge Chain", &HingeChain::Create },
	{ "Ragdoll", &Ragdoll::Create },
	{ "Newton's Cradle", &NewtonCradle::Create },
	{ "Mesh Contact Test", &MeshContactTest::Create },
	{ "Sphere Stack", &SphereStack::Create },
	{ "Capsule Stack", &CapsuleStack::Create },
	{ "Box Stack", &BoxStack::Create },
	{ "Shape Stack", &ShapeStack::Create },
	{ "Jenga", &Jenga::Create },
	{ "Box Pyramid", &Pyramid::Create },
	{ "Box Pyramid Rows", &Pyramids::Create },
	{ "Ray Cast", &RayCast::Create },
	{ "Sensor Test", &SensorTest::Create },
	{ "Character Test", &Character::Create },
	{ "Body Types", &BodyTypes::Create },
	{ "Varying Friction", &VaryingFriction::Create },
	{ "Varying Restitution", &VaryingRestitution::Create },
	{ "Cloth", &Cloth::Create },
	{ "Tumbler", &Tumbler::Create },
	{ "Initial Overlap", &InitialOverlap::Create },
	{ NULL, NULL }
};