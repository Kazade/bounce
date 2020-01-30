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

#ifndef BOX_EDGE_CONTACT_H
#define BOX_EDGE_CONTACT_H

class BoxEdgeContact : public Collide
{
public:
	BoxEdgeContact()
	{
		m_box.SetExtents(1.0f, 2.0f, 1.0f);

		m_sA.m_hull = &m_box;
		m_xfA.translation.Set(1.500000, 1.000000, 0.000000);
		m_xfA.rotation.SetIdentity();
		m_shapeA = &m_sA;

		m_sB.m_hull = &m_box;
		m_xfB.translation.Set(-1.29999995, 1.34999979, 0.000000000);
		m_xfB.rotation.Set(0.810514629, 0.342624813, 0.334119707, 0.337692767);
		m_shapeB = &m_sB;

		m_cache.count = 0;
	}
	
	static Test* Create()
	{
		return new BoxEdgeContact();
	}

	b3BoxHull m_box;
	b3HullShape m_sA;
	b3HullShape m_sB;
};

#endif
