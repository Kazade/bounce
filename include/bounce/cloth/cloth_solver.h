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

#ifndef B3_CLOTH_SOLVER_H
#define B3_CLOTH_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>

class b3StackAllocator;

class b3ClothParticle;
class b3Force;
class b3ClothSphereAndShapeContact;
class b3ClothSphereAndTriangleContact;
class b3ClothCapsuleAndCapsuleContact;

struct b3ClothTimeStep;

struct b3ClothSolverDef
{
	b3StackAllocator* stack;
	u32 particleCapacity;
	u32 forceCapacity;
	u32 shapeContactCapacity;
	u32 triangleContactCapacity;
	u32 capsuleContactCapacity;
};

class b3ClothSolver
{
public:
	b3ClothSolver(const b3ClothSolverDef& def);
	~b3ClothSolver();
	
	void Add(b3ClothParticle* p);
	void Add(b3Force* f);
	void Add(b3ClothSphereAndShapeContact* c);
	void Add(b3ClothSphereAndTriangleContact* c);
	void Add(b3ClothCapsuleAndCapsuleContact* c);

	void Solve(const b3ClothTimeStep& step, const b3Vec3& gravity);
private:
	b3StackAllocator* m_stack;

	u32 m_particleCapacity;
	u32 m_particleCount;
	b3ClothParticle** m_particles;

	u32 m_forceCapacity;
	u32 m_forceCount;
	b3Force** m_forces;

	u32 m_shapeContactCapacity;
	u32 m_shapeContactCount;
	b3ClothSphereAndShapeContact** m_shapeContacts;

	u32 m_triangleContactCapacity;
	u32 m_triangleContactCount;
	b3ClothSphereAndTriangleContact** m_triangleContacts;
	
	u32 m_capsuleContactCapacity;
	u32 m_capsuleContactCount;
	b3ClothCapsuleAndCapsuleContact** m_capsuleContacts;
};

#endif