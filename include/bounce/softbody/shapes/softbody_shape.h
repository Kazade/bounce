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

#ifndef B3_SOFTBODY_SHAPE_H
#define B3_SOFTBODY_SHAPE_H

#include <bounce/common/settings.h>

class b3SoftBody;

// Soft body shape type
enum b3SoftBodyShapeType
{
	e_softBodySphereShape,
	e_softBodyWorldShape,
	e_maxSoftBodyShapes
};

struct b3SoftBodyShapeDef
{
	b3SoftBodyShapeDef()
	{
		density = scalar(0);
		radius = scalar(0);
		friction = scalar(0);
	}
	
	// Density
	scalar density;

	// Radius
	scalar radius;

	// Coefficient of friction
	scalar friction;

};

// Soft body shape
class b3SoftBodyShape
{
public:
protected:
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodyContactSolver;

	// Type
	b3SoftBodyShapeType m_type;

	// Density
	scalar m_density;

	// Radius
	scalar m_radius;

	// Coefficient of friction
	scalar m_friction;

	// Broadphase ID
	u32 m_broadPhaseId;

	// Soft body
	b3SoftBody* m_body;
};

#endif