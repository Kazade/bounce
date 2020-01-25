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

#ifndef B3_SOFT_BODY_ANCHOR_H
#define B3_SOFT_BODY_ANCHOR_H

#include <bounce/common/template/list.h>
#include <bounce/common/math/mat33.h>

class b3Body;
class b3SoftBodyNode;

struct b3SoftBodySolverData;

// Soft body anchor joint definition.
// This requires defining local anchor points.
struct b3SoftBodyAnchorDef
{
	// Initialize this joint using body, node and an anchor point in world coordinates.
	void Initialize(b3Body* bodyA, b3SoftBodyNode* nodeB, const b3Vec3& anchor);

	// Body A
	b3Body* bodyA;
	
	// Node B
	b3SoftBodyNode* nodeB;
	
	// The anchor point relative to body A's origin
	b3Vec3 localAnchorA;
	
	// The anchor point relative to node B's origin
	b3Vec3 localAnchorB;
};

// Use soft body anchors to attach a body to a soft body node with bilateral response.
class b3SoftBodyAnchor
{
public:
	// Get the world anchor point A.
	b3Vec3 GetAnchorA() const;

	// Get the world anchor point B.
	b3Vec3 GetAnchorB() const;
	
	// Get the local anchor point relative to body A's origin.
	const b3Vec3& GetLocalAnchorA() const { return m_localAnchorA; }

	// Get the local anchor point relative to node B's origin.
	const b3Vec3& GetLocalAnchorB() const { return m_localAnchorB; }
private:
	friend class b3SoftBody;
	friend class b3SoftBodySolver;
	friend class b3List2<b3SoftBodyAnchor>;

	b3SoftBodyAnchor(const b3SoftBodyAnchorDef& def);
	~b3SoftBodyAnchor() { }
	
	void InitializeConstraints(const b3SoftBodySolverData* data);
	void WarmStart(const b3SoftBodySolverData* data);
	void SolveVelocityConstraints(const b3SoftBodySolverData* data);
	bool SolvePositionConstraints(const b3SoftBodySolverData* data);

	// Solver shared
	b3Body* m_bodyA;
	b3SoftBodyNode* m_nodeB;

	b3Vec3 m_localAnchorA;
	b3Vec3 m_localAnchorB;

	// Solver temp
	scalar m_mA;
	b3Mat33 m_iA;
	
	scalar m_mB;
	u32 m_indexB;

	b3Vec3 m_rA;

	b3Mat33 m_mass;
	b3Vec3 m_impulse;
	b3Vec3 m_velocityBias;

	b3SoftBodyAnchor* m_prev;
	b3SoftBodyAnchor* m_next;
};

#endif