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
	float32 m_x;

	b3SoftBody* m_body;
	const b3SoftBodyMesh* m_mesh;
	const b3SoftBodyMeshTetrahedron* m_tetrahedron;
	u32 m_v1, m_v2, m_v3, m_v4;
	float32 m_tu, m_tv, m_tw, m_tx;
};

inline bool b3SoftBodyDragger::IsDragging() const
{
	return m_tetrahedron != nullptr;
}

#endif