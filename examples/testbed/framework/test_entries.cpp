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

#include <testbed/framework/test.h>
#include <testbed/tests/convex_hull.h>
#include <testbed/tests/cluster.h>
#include <testbed/tests/distance_test.h>
#include <testbed/tests/linear_time_of_impact.h>
#include <testbed/tests/time_of_impact.h>
#include <testbed/tests/aabb_time_of_impact.h>
#include <testbed/tests/collide_test.h>
#include <testbed/tests/capsule_collision.h>
#include <testbed/tests/hull_collision.h>
#include <testbed/tests/deep_capsule.h>
#include <testbed/tests/box_face_contact.h>
#include <testbed/tests/box_edge_contact.h>
#include <testbed/tests/linear_motion.h>
#include <testbed/tests/angular_motion.h>
#include <testbed/tests/gyro_motion.h>
#include <testbed/tests/initial_overlap.h>
#include <testbed/tests/capsule_spin.h>
#include <testbed/tests/quadric_shapes.h>
#include <testbed/tests/compound_body.h>
#include <testbed/tests/spring_test.h>
#include <testbed/tests/motor_test.h>
#include <testbed/tests/weld_test.h>
#include <testbed/tests/cone_test.h>
#include <testbed/tests/revolute_test.h>
#include <testbed/tests/prismatic_test.h>
#include <testbed/tests/wheel_test.h>
#include <testbed/tests/hinge_chain.h>
#include <testbed/tests/newton_cradle.h>
#include <testbed/tests/ragdoll.h>
#include <testbed/tests/mesh_contact_test.h>
#include <testbed/tests/triangle_contact_test.h>
#include <testbed/tests/hull_contact_test.h>
#include <testbed/tests/sphere_stack.h>
#include <testbed/tests/capsule_stack.h>
#include <testbed/tests/box_stack.h>
#include <testbed/tests/sheet_stack.h>
#include <testbed/tests/shape_stack.h>
#include <testbed/tests/jenga.h>
#include <testbed/tests/pyramid.h>
#include <testbed/tests/pyramids.h>
#include <testbed/tests/ray_cast.h>
#include <testbed/tests/convex_cast.h>
#include <testbed/tests/sensor_test.h>
#include <testbed/tests/body_types.h>
#include <testbed/tests/varying_friction.h>
#include <testbed/tests/varying_restitution.h>
#include <testbed/tests/tumbler.h>
#include <testbed/tests/multiple_pendulum.h>
#include <testbed/tests/conveyor_belt.h>
#include <testbed/tests/table_cloth.h>
#include <testbed/tests/cloth_sdf.h>
#include <testbed/tests/pinned_cloth.h>
#include <testbed/tests/particle_types.h>
#include <testbed/tests/tension_mapping.h>
#include <testbed/tests/cloth_self_collision.h>
#include <testbed/tests/cape.h>
#include <testbed/tests/cloth_tearing.h>
#include <testbed/tests/cloth_element_test.h>
#include <testbed/tests/rope_test.h>
#include <testbed/tests/beam.h>
#include <testbed/tests/sheet.h>
#include <testbed/tests/node_types.h>
#include <testbed/tests/pinned_softbody.h>
#include <testbed/tests/softbody_anchor.h>
#include <testbed/tests/smash_softbody.h>
#include <testbed/tests/tetgen_softbody.h>

TestEntry g_tests[] =
{
	{ "Convex Hull", &ConvexHull::Create },
	{ "Cluster", &Cluster::Create },
	{ "Distance", &Distance::Create },
	{ "Linear Time of Impact", &LinearTimeOfImpact::Create },
	{ "Time of Impact", &TimeOfImpact::Create },
	{ "AABB Time of Impact", &AABBTimeOfImpact::Create },
	{ "Capsule Collision", &CapsuleCollision::Create },
	{ "Hull Collision", &HullCollision::Create },
	{ "Deep Capsule", &DeepCapsule::Create },
	{ "Box Face Contact", &BoxFaceContact::Create },
	{ "Box Edge Contact", &BoxEdgeContact::Create },
	{ "Capsule Spin", &CapsuleSpin::Create },
	{ "Hull Contact Test", &HullContactTest::Create },
	{ "Triangle Contact Test", &TriangleContactTest::Create },
	{ "Mesh Contact Test", &MeshContactTest::Create },
	{ "Linear Motion", &LinearMotion::Create },
	{ "Angular Motion", &AngularMotion::Create },
	{ "Gyroscopic Motion", &GyroMotion::Create },
	{ "Compound Body", &CompoundBody::Create },
	{ "Quadric Shapes", &QuadricShapes::Create },
	{ "Spring Test", &SpringTest::Create },
	{ "Prismatic Test", &PrismaticTest::Create },
	{ "Wheel Test", &WheelTest::Create },
	{ "Weld Test", &WeldTest::Create },
	{ "Cone Test", &ConeTest::Create },
	{ "Motor Test", &MotorTest::Create },
	{ "Revolute Test", &RevoluteTest::Create },
	{ "Hinge Chain", &HingeChain::Create },
	{ "Ragdoll", &Ragdoll::Create },
	{ "Newton's Cradle", &NewtonCradle::Create },
	{ "Sphere Stack", &SphereStack::Create },
	{ "Capsule Stack", &CapsuleStack::Create },
	{ "Box Stack", &BoxStack::Create },
	{ "Sheet Stack", &SheetStack::Create },
	{ "Shape Stack", &ShapeStack::Create },
	{ "Jenga", &Jenga::Create },
	{ "Box Pyramid", &Pyramid::Create },
	{ "Box Pyramid Rows", &Pyramids::Create },
	{ "Ray Cast", &RayCast::Create },
	{ "Convex Cast", &ConvexCast::Create },
	{ "Sensor Test", &SensorTest::Create },
	{ "Body Types", &BodyTypes::Create },
	{ "Varying Friction", &VaryingFriction::Create },
	{ "Varying Restitution", &VaryingRestitution::Create },
	{ "Tumbler", &Tumbler::Create },
	{ "Initial Overlap", &InitialOverlap::Create },
	{ "Multiple Pendulum", &MultiplePendulum::Create },
	{ "Conveyor Belt", &ConveyorBelt::Create },
	{ "Table Cloth", &TableCloth::Create },
	{ "Cloth SDF", &ClothSDF::Create },
	{ "Pinned Cloth", &PinnedCloth::Create },
	{ "Particle Types", &ParticleTypes::Create },
	{ "Tension Mapping", &TensionMapping::Create },
	{ "Cloth Self-Collision", &ClothSelfCollision::Create },
	{ "Cloth Tearing", &ClothTearing::Create },
	{ "Cloth Element Test", &ClothElementTest::Create },
	{ "Cape", &Cape::Create },
	{ "Beam", &Beam::Create },
	{ "Sheet", &Sheet::Create },
	{ "Node Types", &NodeTypes::Create },
	{ "Pinned Soft Body", &PinnedSoftBody::Create },
	{ "Soft Body Anchor", &SoftBodyAnchor::Create },
	{ "Smash Soft Body", &SmashSoftBody::Create },
	{ "TetGen Soft Body", &TetGenSoftBody::Create },
	{ "Rope", &Rope::Create },
	{ NULL, NULL }
};

//
static u32 TestCount()
{
	u32 count = 0;
	while (g_tests[count].create != NULL)
	{
		++count;
	}
	return count;
}

// Count the tests
u32 g_testCount = TestCount();