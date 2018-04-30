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

#ifndef BOX_EDGE_CONTACT_H
#define BOX_EDGE_CONTACT_H

class BoxEdgeContact : public Collide
{
public:
	BoxEdgeContact()
	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(1.0f, 2.0f, 1.0f);
		m_box.SetTransform(xf);

		m_sA.m_hull = &m_box;
		m_sB.m_hull = &m_box;
		
		m_xfA.position.Set(1.500000, 1.000000, 0.000000);
		m_xfA.rotation.x.Set(0.707107, 0.000000, -0.707107);
		m_xfA.rotation.y.Set(0.000000, 1.000000, 0.000000);
		m_xfA.rotation.z.Set(0.707107, 0.000000, 0.707107);

		m_xfB.position.Set(-1.300000, 0.000000, 0.000000);
		m_xfB.rotation.x.Set(0.809017, 0.266849, -0.523721);
		m_xfB.rotation.y.Set(0.000000, 0.891007, 0.453991);
		m_xfB.rotation.z.Set(0.587785, -0.367286, 0.720840);

		m_cache.count = 0;
		m_shapeA = &m_sA;
		m_shapeB = &m_sB;
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
