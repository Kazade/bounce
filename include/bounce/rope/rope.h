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

#ifndef B3_ROPE_H
#define B3_ROPE_H

#include <bounce/common/math/transform.h>

struct b3RopeBody;

//
struct b3RopeDef
{
	b3RopeDef()
	{
		vertices = NULL;
		masses = NULL;
		count = 0;
		gravity.SetZero();
		linearDamping = 0.6f;
		angularDamping = 0.6f;
	}

	//
	b3Vec3* vertices;

	//
	float32* masses;

	//
	u32 count;

	//
	b3Vec3 gravity;

	//
	float32 linearDamping;

	//
	float32 angularDamping;
};

//
class b3Rope
{
public:
	//
	b3Rope();
	
	//
	~b3Rope();

	//
	void Initialize(const b3RopeDef& def);
	
	//
	void SetOrigin(const b3Vec3& position)
	{
		m_p = position;
	}

	//
	void SetGravity(const b3Vec3& gravity)
	{
		m_gravity = gravity;
	}

	//
	void Step(float32 dt);

	//
	void Draw() const;
private:
	//
	float32 m_kd1, m_kd2;

	//
	b3Vec3 m_gravity;

	// Base
	b3Vec3 m_v;
	b3Vec3 m_w;

	b3Vec3 m_p;
	b3Quat m_q;

	// 
	u32 m_count;
	b3RopeBody* m_links;	
};

#endif