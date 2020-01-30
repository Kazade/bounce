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

#ifndef B3_SOFT_BODY_SOLVER_H
#define B3_SOFT_BODY_SOLVER_H

#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/softbody/softbody_time_step.h>

class b3StackAllocator;

class b3SoftBody;
struct b3SoftBodyMesh;

class b3SoftBodyNode;
class b3SoftBodyElement;

class b3SoftBodySphereAndShapeContact;
class b3SoftBodyAnchor;

struct b3SoftBodySolverDef
{
	b3SoftBody* body;
};

struct b3SoftBodySolverData
{
	b3SoftBodyTimeStep step;
	b3Vec3* positions;
	b3Vec3* velocities;
};

class b3SoftBodySolver
{
public:
	b3SoftBodySolver(const b3SoftBodySolverDef& def);
	~b3SoftBodySolver();

	void Add(b3SoftBodySphereAndShapeContact* c);
	void Add(b3SoftBodyAnchor* a);

	void Solve(const b3SoftBodyTimeStep& step, const b3Vec3& gravity);
private:
	b3SoftBody* m_body;
	b3StackAllocator* m_stack;
	const b3SoftBodyMesh* m_mesh;
	b3SoftBodyNode* m_nodes;
	b3SoftBodyElement* m_elements;
	
	u32 m_shapeContactCapacity;
	u32 m_shapeContactCount;
	b3SoftBodySphereAndShapeContact** m_shapeContacts;
	
	u32 m_anchorCapacity;
	u32 m_anchorCount;
	b3SoftBodyAnchor** m_anchors;
};

#endif