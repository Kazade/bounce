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

#ifndef B3_BODY_DRAGGER_H
#define B3_BODY_DRAGGER_H

#include <bounce/common/geometry.h>
#include <bounce/common/geometry.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/joints/mouse_joint.h>

// A body shape dragger.
class b3BodyDragger
{
public:
	b3BodyDragger(b3Ray3* ray, b3World* world);
	~b3BodyDragger();

	bool StartDragging();

	void Drag();

	void StopDragging();

	bool IsDragging() const;

	b3Ray3* GetRay() const;

	b3Body* GetBody() const;

	b3Vec3 GetPointA() const;

	b3Vec3 GetPointB() const;
private:
	b3Ray3 * m_ray;
	float32 m_x;

	b3World* m_world;
	
	b3Shape* m_shape;
	b3Vec3 m_p;
	b3MouseJoint* m_mouseJoint;
};

inline bool b3BodyDragger::IsDragging() const
{
	return m_shape != nullptr;
}

inline b3Ray3* b3BodyDragger::GetRay() const
{
	return m_ray;
}

#endif