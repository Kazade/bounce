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

#ifndef BOX_FACE_CONTACT_H
#define BOX_FACE_CONTACT_H

class BoxFaceContact : public Collide
{
public:
	BoxFaceContact()
	{
		m_boxA.SetExtents(1.0f, 2.0f, 1.0f);
		b3Vec3 translation(0.0f, 2.0f, 0.0f);
		m_boxA.Translate(translation);

		m_sA.m_hull = &m_boxA;
		
		m_xfA.SetIdentity();

		m_shapeA = &m_sA;
		
		m_boxB.SetExtents(1.0f, 1.0f, 1.0f);

		m_xfB.SetIdentity();
		m_sB.m_hull = &m_boxB;

		m_shapeB = &m_sB;
		
		m_cache.count = 0;
	}

	static Test* Create()
	{
		return new BoxFaceContact();
	}

	b3BoxHull m_boxA;
	b3BoxHull m_boxB;
	b3HullShape m_sA;
	b3HullShape m_sB;
};

#endif
