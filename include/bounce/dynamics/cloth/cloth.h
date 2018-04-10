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

#ifndef B3_CLOTH_H
#define B3_CLOTH_H

#include <bounce/common/math/vec3.h>
#include <bounce/collision/collision.h>

struct b3Mesh;

struct b3ClothDef
{
	b3ClothDef()
	{
		mesh = NULL;
		density = 0.0f;
		gravity.SetZero();
		k1 = 0.9f;
		k2 = 0.2f;
		kd = 0.1f;
		r = 0.0f;
	}

	// Cloth mesh
	// Each edge must be shared by at most two triangles (manifold)
	const b3Mesh* mesh;

	// Cloth density in kilograms per meter squared
	float32 density;

	// Gravity force
	b3Vec3 gravity;

	// Streching stiffness
	float32 k1;

	// Bending stiffness
	float32 k2;
	
	// Damping
	float32 kd;

	// Cloth thickness
	float32 r;
};

struct b3Particle
{
	float32 im;
	b3Vec3 p0;
	b3Vec3 p;
	b3Vec3 v;
};

struct b3C1
{
	float32 L;
	u32 i1;
	u32 i2;
};

struct b3C2
{
	float32 angle;
	u32 i1;
	u32 i2;
	u32 i3;
	u32 i4;
};

class b3Cloth
{
public:
	b3Cloth();
	~b3Cloth();

	void Initialize(const b3ClothDef& def);

	void Step(float32 dt, u32 iterations);

	u32 GetVertexCount() const
	{
		return m_pCount;
	}

	const b3Particle* GetVertices() const
	{
		return m_ps;
	}

	b3Particle* GetVertices()
	{
		return m_ps;
	}

	void Draw() const;
private:
	void SolveC1();
	void SolveC2();

	b3Particle* m_ps;
	u32 m_pCount;
	
	b3C1* m_c1s;
	u32 m_c1Count;
	
	b3C2* m_c2s;
	u32 m_c2Count;

	float32 m_k1;
	float32 m_k2;
	float32 m_kd;
	float32 m_r;
	b3Vec3 m_gravity;

	const b3Mesh* m_mesh;
};

#endif