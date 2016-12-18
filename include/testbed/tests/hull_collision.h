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

#ifndef HULL_HULL_H
#define HULL_HULL_H

class HullAndHull : public Collide
{
public:
	HullAndHull()
	{
		b3Transform xf;
		xf.rotation = b3Diagonal(1.0f, 2.0f, 1.0f);
		xf.position.SetZero();

		m_box.SetTransform(xf);

		b3Quat qA(0.0f, 1.0f, 0.0f, 0.025f * B3_PI);
		m_xfA.SetIdentity();
		m_xfA.position.Set(0.0186814368f, 1.96078217f, 0.0253920462f);
		m_xfA.rotation = b3ConvertQuatToRot(qA);
		m_sA.m_hull = &m_box;
		
		m_xfB.SetIdentity();
		m_xfB.position.Set(0.f, 0.0f, 0.0f);
		m_sB.m_hull = &m_box;

		m_cache.count = 0;
		m_shapeA = &m_sA;
		m_shapeB = &m_sB;
	}

	static Test* Create()
	{
		return new HullAndHull();
	}

	b3BoxHull m_box;
	b3HullShape m_sA;
	b3HullShape m_sB;
};

#endif
