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

#include <bounce/cloth/cloth_triangle.h>
#include <bounce/cloth/cloth.h>
#include <bounce/cloth/cloth_mesh.h>

void b3ClothTriangle::Synchronize(const b3Vec3& displacement)
{
	b3ClothMeshTriangle* triangle = m_cloth->m_mesh->triangles + m_triangle;

	b3Particle* p1 = m_cloth->m_particles[triangle->v1];
	b3Particle* p2 = m_cloth->m_particles[triangle->v2];
	b3Particle* p3 = m_cloth->m_particles[triangle->v3];

	b3Vec3 x1 = p1->m_position;
	b3Vec3 x2 = p2->m_position;
	b3Vec3 x3 = p3->m_position;

	b3AABB3 aabb;
	aabb.Set(x1, x2, x3);
	aabb.Extend(m_radius);

	m_cloth->m_contactManager.m_broadPhase.MoveProxy(m_broadPhaseId, aabb, displacement);
}