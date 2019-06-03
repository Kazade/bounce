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

#include <bounce/softbody/softbody_node.h>
#include <bounce/softbody/softbody.h>

void b3NodeBodyContactWorldPoint::Initialize(const b3NodeBodyContact* c, float32 rA, const b3Transform& xfA, float32 rB, const b3Transform& xfB)
{
	b3Vec3 nA = c->normal1;

	b3Vec3 cA = xfA * c->localPoint1;
	b3Vec3 cB = xfB * c->localPoint2;

	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = 0.5f * (pA + pB);
	normal = nA;
	separation = b3Dot(cB - cA, nA) - rA - rB;
}

void b3SoftBodyNode::Synchronize()
{
	b3AABB3 aabb;
	aabb.Set(m_position, m_radius);

	m_body->m_nodeTree.UpdateNode(m_treeId, aabb);
}