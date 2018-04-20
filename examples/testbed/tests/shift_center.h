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

#ifndef SHIFT_CENTER_H
#define SHIFT_CENTER_H

class ShiftCenter : public Test
{
public:
	ShiftCenter()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3ShapeDef sd;
			sd.shape = &hs;

			ground->CreateShape(sd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape sphere;
			sphere.m_center.SetZero();
			sphere.m_radius = 1.0f;

			b3ShapeDef sd;
			sd.density = 0.1f;
			sd.friction = 0.3f;
			sd.shape = &sphere;

			body->CreateShape(sd);

			// Retrieve the local center of mass and inertia about the 
			// local center of mass
			b3MassData massData;
			body->GetMassData(&massData);

			// Shift the inertia to the local origin
			massData.I += massData.mass * b3Steiner(massData.center);

			// Make a copy of the old local center of mass
			b3Vec3 oldCenter = massData.center;
			
			// Pick a center of mass of choice
			massData.center.z += 10.0f;

			// Measure the displacement from the old local center of mass 
			// to the new local center of mass.
			b3Vec3 d = massData.center - oldCenter;

			// Move the inertia at the local origin to the new local origin
			massData.I += massData.mass * b3Steiner(d);

			// Update local center of mass and inertia
			body->SetMassData(&massData);
		}
	}

	~ShiftCenter()
	{
	}

	static Test* Create()
	{
		return new ShiftCenter();
	}
};

#endif