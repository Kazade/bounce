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

#ifndef B3_SOFTBODY_DRAGGER_H
#define B3_SOFTBODY_DRAGGER_H

#include <bounce/common/geometry.h>
#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>

// A soft body triangle dragger.
class b3SoftBodyDragger
{
public:
	b3SoftBodyDragger(b3Ray3* ray, b3SoftBody* body);
	~b3SoftBodyDragger();

	bool IsDragging() const;

	bool StartDragging();

	void Drag();

	void StopDragging();

	b3Vec3 GetPointA() const;

	b3Vec3 GetPointB() const;
private:
	b3Ray3* m_ray;
	b3SoftBody* m_body;

	bool m_isDragging;
	scalar m_x;
	scalar m_tu, m_tv, m_tw;
	b3SoftBodyNode * m_n1, * m_n2, * m_n3;
	b3SoftBodyNodeType m_t1, m_t2, m_t3;
};

inline bool b3SoftBodyDragger::IsDragging() const
{
	return m_isDragging;
}

#endif