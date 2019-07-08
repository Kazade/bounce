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

#include <bounce/bounce.h>
#include <stdio.h>

// We don't care for a profiler. This definition does nothing.
void b3BeginProfileScope(const char* name)
{

}

// We don't care for a profiler. This definition does nothing.
void b3EndProfileScope()
{
	
}

// This example shows how to setup and run a simple simulation 
// using Bounce. 
int main(int argc, char** argv)
{
	// The world. We allocate it using the heap but you can to it 
	// on the stack if the stack is sufficiently large.
	b3World* world = new b3World();

	// The world gravity.
	const b3Vec3 gravity(0.0f, -9.8f, 0.0f);
	
	world->SetGravity(gravity);
	
	// The fixed time step size.
	const float32 timeStep = 1.0f / 60.0f;
	
	// Number of iterations for the velocity constraint solver.
	const u32 velocityIterations = 8;

	// Number of iterations for the position constraint solver.
	const u32 positionIterations = 2;

	// Create a static ground body at the world origin.
	b3BodyDef groundDef;
	b3Body* ground = world->CreateBody(groundDef);

	// Create a box positioned at the world origin and 
	// aligned with the world frame.
	b3BoxHull groundBox;
	
	// Set the ground box dimensions using a linear scale transform.
	b3Transform scale;
	scale.position.SetZero();
	scale.rotation = b3Diagonal(10.0f, 1.0f, 10.0f);
	groundBox.SetTransform(scale);
	
	// Create the box physics wrapper.
	b3HullShape groundShape;
	groundShape.m_hull = &groundBox;

	// Add the box to the ground body.
	b3ShapeDef groundBoxDef;
	groundBoxDef.shape = &groundShape;
	ground->CreateShape(groundBoxDef);

	// Create a dynamic body.
	b3BodyDef bodyDef;
	bodyDef.type = e_dynamicBody;
	
	// Position the body 10 meters high from the world origin.
	bodyDef.position.Set(0.0f, 10.0f, 0.0f);
	
	// Set the initial angular velocity to pi radians (180 degrees) per second.
	bodyDef.angularVelocity.Set(0.0f, B3_PI, 0.0f);
	
	b3Body* body = world->CreateBody(bodyDef);

	// Create a unit box positioned at the world origin and 
	// aligned with the world frame.
	b3BoxHull bodyBox;
	bodyBox.SetIdentity();

	// Create the box physics wrapper.
	b3HullShape bodyShape;
	bodyShape.m_hull = &bodyBox;

	// Add the box to the body.
	b3ShapeDef bodyBoxDef;
	bodyBoxDef.shape = &bodyShape;
	bodyBoxDef.density = 1.0f;
	body->CreateShape(bodyBoxDef);

	// Run a small game loop of 60 frames length.
	for (u32 i = 0; i < 60; ++i)
	{
		// Perform a time step of the world in this frame.
		world->Step(timeStep, velocityIterations, positionIterations);
		
		// Read the body position and orientation in this frame.
		b3Vec3 position = body->GetPosition();
		b3Quat orientation = body->GetOrientation();
		
		// Decode the axis and angle of rotation about it from the quaternion.
		b3Vec3 axis;
		float32 angle;
		orientation.GetAxisAngle(&axis, &angle);

		// Visualize the body state in this frame.
		printf("position = %.2f %.2f %.2f\n", position.x, position.y, position.z);
		printf("axis = %.2f %.2f %.2f, angle = %.2f\n\n", axis.x, axis.y, axis.z, angle);
	}
	
	// Now destroy the bodies since the world manages their lifetime.
	delete world;

	return 0;
}