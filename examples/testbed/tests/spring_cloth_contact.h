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

#ifndef SPRING_CLOTH_CONTACT_H
#define SPRING_CLOTH_CONTACT_H

class SpringClothContact : public SpringClothTest
{
public:
	SpringClothContact()
	{
		b3SpringClothDef def;
		def.allocator = &m_clothAllocator;
		def.mesh = &m_clothMesh;
		def.density = 0.2f;
		def.ks = 1000.0f;
		def.kd = 0.0f;
		def.r = 0.2f;
		def.gravity.Set(0.0f, -10.0f, 0.0f);

		m_cloth.Initialize(def);

		m_clothCapsule.m_centers[0].Set(0.0f, -2.0f, 2.0f);
		m_clothCapsule.m_centers[1].Set(0.0f, -2.0f, -2.0f);
		m_clothCapsule.m_radius = 2.0f;
		
		m_clothCapsule.SetFriction(1.0f);

		m_cloth.AddShape(&m_clothCapsule);
	}

	static Test* Create()
	{
		return new SpringClothContact();
	}

	b3GridMesh<10, 10> m_clothMesh;
	b3CapsuleShape m_clothCapsule;
};

#endif