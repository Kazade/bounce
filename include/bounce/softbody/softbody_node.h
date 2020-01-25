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

#ifndef B3_SOFT_BODY_NODE_H
#define B3_SOFT_BODY_NODE_H

#include <bounce/common/math/vec3.h>

class b3SoftBody;

// Static node: Can be moved manually.
// Kinematic node: Non-zero velocity, can be moved by the solver.
// Dynamic node: Non-zero velocity determined by force, can be moved by the solver.
enum b3SoftBodyNodeType
{
	e_staticSoftBodyNode,
	e_kinematicSoftBodyNode,
	e_dynamicSoftBodyNode
};

// A soft body node.
class b3SoftBodyNode 
{
public:
	// Set the node type.
	void SetType(b3SoftBodyNodeType type);

	// Get the node type.
	b3SoftBodyNodeType GetType() const;

	// Set the node position. 
	// If the node is dynamic changing the position directly might lead 
	// to physically incorrect simulation behaviour.
	void SetPosition(const b3Vec3& position);

	// Get the node position.
	const b3Vec3& GetPosition() const;

	// Set the node velocity.
	void SetVelocity(const b3Vec3& velocity);

	// Get the node velocity.
	const b3Vec3& GetVelocity() const;

	// Get the node mass.
	scalar GetMass() const;

	// Get the mesh vertex index.
	u32 GetMeshIndex() const;

	// Apply a force.
	void ApplyForce(const b3Vec3& force);

	// Get the applied force.
	const b3Vec3& GetForce() const;

	// Apply a translation.
	void ApplyTranslation(const b3Vec3& translation);

	// Get the applied translation.
	const b3Vec3& GetTranslation() const;
private:
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyForceSolver;
	friend class b3SoftBodyContactSolver;
	friend class b3SoftBodySphereAndShapeContact;
	friend class b3SoftBodyAnchor;

	b3SoftBodyNode() { }
	~b3SoftBodyNode() { }

	// Synchronize spheres
	void SynchronizeSpheres();

	// Destroy contacts
	void DestroyContacts();
	
	// Type
	b3SoftBodyNodeType m_type;

	// Position
	b3Vec3 m_position;
	
	// Velocity
	b3Vec3 m_velocity;

	// Applied external force
	b3Vec3 m_force;

	// Applied external translation
	b3Vec3 m_translation;
	
	// Mass
	scalar m_mass;

	// Inverse mass
	scalar m_invMass;

	// Mesh index
	u32 m_meshIndex;

	// Soft body
	b3SoftBody* m_body;
};

inline b3SoftBodyNodeType b3SoftBodyNode::GetType() const
{
	return m_type;
}

inline void b3SoftBodyNode::SetPosition(const b3Vec3& position)
{
	m_position = position;
	SynchronizeSpheres();
}

inline const b3Vec3& b3SoftBodyNode::GetPosition() const
{
	return m_position;
}

inline void b3SoftBodyNode::SetVelocity(const b3Vec3& velocity)
{
	if (m_type == e_staticSoftBodyNode)
	{
		return;
	}
	m_velocity = velocity;
}

inline const b3Vec3& b3SoftBodyNode::GetVelocity() const
{
	return m_velocity;
}

inline scalar b3SoftBodyNode::GetMass() const
{
	return m_mass;
}

inline void b3SoftBodyNode::ApplyForce(const b3Vec3& force)
{
	if (m_type != e_dynamicSoftBodyNode)
	{
		return;
	}
	m_force += force;
}

inline const b3Vec3& b3SoftBodyNode::GetForce() const
{
	return m_force;
}

inline void b3SoftBodyNode::ApplyTranslation(const b3Vec3& translation)
{
	m_translation += translation;
}

inline const b3Vec3& b3SoftBodyNode::GetTranslation() const
{
	return m_translation;
}

inline u32 b3SoftBodyNode::GetMeshIndex() const
{
	return m_meshIndex;
}

#endif