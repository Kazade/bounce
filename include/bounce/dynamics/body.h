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

#ifndef B3_BODY_H
#define B3_BODY_H

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>
#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>
#include <bounce/dynamics/time_step.h>

class b3World;
class b3Shape;

struct b3ShapeDef;
struct b3MassData;
struct b3JointEdge;

// Static body: Has zero mass, can be moved manually.
// Kinematic body: Has zero mass, non-zero velocity, can be moved by the solver.
// Dynamic body: Has non-zero mass, non-zero velocity determined by force, can be moved by the solver.
enum b3BodyType 
{
	e_staticBody,
	e_kinematicBody,
	e_dynamicBody
};

// Pass this definition to the world to create a new rigid body.
struct b3BodyDef 
{
	b3BodyDef() 
	{
		type = e_staticBody;
		awake = true;
		fixedRotationX = false;
		fixedRotationY = false;
		fixedRotationZ = false;
		userData = NULL;
		position.SetZero();
		orientation.SetIdentity();
		linearVelocity.SetZero();
		angularVelocity.SetZero();
		gravityScale = 1.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
	}

	//
	b3BodyType type;
	
	//
	bool awake;
	
	//
	bool fixedRotationX;
	
	//
	bool fixedRotationY;
	
	//
	bool fixedRotationZ;
	
	//
	void* userData;
	
	//
	b3Vec3 position;
	
	//
	b3Quat orientation;
	
	//
	b3Vec3 linearVelocity;
	
	//
	b3Vec3 angularVelocity;
	
	//
	float32 linearDamping;

	//
	float32 angularDamping;

	//
	float32 gravityScale;
};

class b3Body
{
public:
	// Get the type of the body.
	b3BodyType GetType() const;

	// Set the type of the body. 
	// This will reset the current body inertial properties.
	void SetType(b3BodyType type);

	// Get the world the body belongs to.
	const b3World* GetWorld() const;
	b3World* GetWorld();

	// Get the shapes associated with the body.
	const b3List1<b3Shape>& GetShapeList() const;
	b3List1<b3Shape>& GetShapeList();

	// Create a new shape for the body given the shape definition and return a pointer to its clone.
	// The shape passed to the definition it will be cloned and is not recommended modifying 
	// it inside simulation callbacks. 
	// Therefore you can create shapes on the stack memory.
	b3Shape* CreateShape(const b3ShapeDef& def);
	
	// Destroy a given shape from the body.
	void DestroyShape(b3Shape* shape);

	// Get the body sweep.
	const b3Sweep& GetSweep() const;

	// Get the body world transform.
	const b3Transform& GetTransform() const;

	// Set the body world transform from a position, axis of rotation and an angle 
	// of rotation about the axis.
	// However, manipulating a body transform during the simulation may cause non-physical behaviour.
	void SetTransform(const b3Vec3& position, const b3Vec3& axis, float32 angle);

	// Get the position of the world body origin.
	b3Vec3 GetPosition() const;

	// Get the orientation of the world body frame.
	b3Quat GetOrientation() const;

	// Get the world position of the center of mass.
	b3Vec3 GetWorldCenter() const;

	// Get the local position of the center of mass.
	b3Vec3 GetLocalCenter() const;
	
	// Get the velocity at a given point.
	b3Vec3 GetPointVelocity(const b3Vec3& point) const;

	// Get the linear velocity of the center of mass.
	b3Vec3 GetLinearVelocity() const;

	// Set the linear velocity of the center of mass. 
	// If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetLinearVelocity(const b3Vec3& linearVelocity);
	
	// Get the angular velocity of the body frame.
	b3Vec3 GetAngularVelocity() const;

	// Set the angular velocity of the body frame. 
	// If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetAngularVelocity(const b3Vec3& angularVelocity);

	// Apply a force at a specific world point. 
	// If the point isn't the center of mass then a non-zero torque is applied.
	// The body must be dynamic.
	void ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake);

	// Apply a force at the center of mass. Usually this is a external force. 
	// The body must be dynamic.
	void ApplyForceToCenter(const b3Vec3& force, bool wake);

	// Apply a torque to the body.
	// The body must be dynamic.
	void ApplyTorque(const b3Vec3& torque, bool wake);

	// Apply a linear impulse (linear velocity change) to a specific point (particle) of the body. 
	// If the point isn't the center of mass then a non-zero angular impulse is applied.
	// The body must be dynamic.
	void ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& point, bool wake);

	// Apply a angular impulse (angular velocity change) to the body.
	// The body must be dynamic.
	void ApplyAngularImpulse(const b3Vec3& impulse, bool wake);

	// Get the mass of the body. Typically in kg/m^3.
	float32 GetMass() const;

	// Get the inverse mass of the body. Typically in kg/m^3.
	float32 GetInverseMass() const;
	
	// Get the rotational inertia of the body about the local center of mass. Typically in kg/m^3.
	const b3Mat33& GetInertia() const;

	// Get the inverse of the rotational inertia of the body about the world center of mass. Typically in kg/m^3.
	const b3Mat33& GetWorldInverseInertia() const;
	
	// Get this body mass data. 
	// However, the mass data returned by this function contains the mass of the body, 
	// the body local center of mass, and the rotational inertia about the body local center of mass.
	void GetMassData(b3MassData* data) const;

	// Set this body mass data.
	// The supplied mass data must have the new mass of the body, local center of mass of the body,
	// and rotational inertia of the body about the body origin. The last requirement must be fulfilled
	// because the given inertia is shifted to the given local center of mass inside this function.
	void SetMassData(const b3MassData* data);

	// Recalculate this body mass data based on all of its shapes.
	void ResetMass();

	// Get the linear kinetic energy of the body in Joules (kg m^2/s^2).
	float32 GetLinearEnergy() const;

	// Get the angular kinetic energy of the body in Joules (kg m^2/s^2).
	float32 GetAngularEnergy() const;

	// Get the total kinetic energy of the body in Joules (kg m^2/s^2).
	float32 GetEnergy() const;
	
	// Transform a vector to the local space of this body.
	b3Vec3 GetLocalVector(const b3Vec3& vector) const;

	// Transform a vector to the world space.
	b3Vec3 GetWorldVector(const b3Vec3& localVector) const;

	// Transform a point to the local space of this body.
	b3Vec3 GetLocalPoint(const b3Vec3& point) const;

	// Transform a point to the world space.
	b3Vec3 GetWorldPoint(const b3Vec3& localPoint) const;

	// Transform a frame to the local space of this body.
	b3Transform GetLocalFrame(const b3Transform& frame) const;

	// Transform a frame to the world space.
	b3Transform GetWorldFrame(const b3Transform& localFrame) const;

	// Get the linear damping of the body. 
	float32 GetLinearDamping() const;
	
	// Set the linear damping of the body. Zero is set by default.
	void SetLinearDamping(float32 damping);

	// Get the angular damping of the body. 
	float32 GetAngularDamping() const;

	// Set the angular damping of the body. Zero is set by default.
	void SetAngularDamping(float32 damping);

	// Get the gravity scale of the body. 
	float32 GetGravityScale() const;

	// Set the gravity scale of the body. One is set by default.
	void SetGravityScale(float32 scale);

	// See if the body is awake.
	bool IsAwake() const;

	// Set the awake status of the body.
	void SetAwake(bool flag);

	// Get the user data associated with the body.
	// The user data is usually a game entity.
	void* GetUserData() const;

	// Set the user data to the body.
	void SetUserData(void* _userData);

	// Get the next body in the world body list.
	const b3Body* GetNext() const;
	b3Body* GetNext();

	// Dump this body to a file.
	void Dump() const;
private:
	friend class b3World;
	friend class b3Island;

	friend class b3Contact;
	friend class b3ConvexContact;
	friend class b3MeshContact;
	friend class b3ContactManager;
	friend class b3ContactSolver;
	friend class b3ClothSolver;
	friend class b3ClothContactSolver;

	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;
	friend class b3MouseJoint;
	friend class b3SpringJoint;
	friend class b3WeldJoint;
	friend class b3RevoluteJoint;
	friend class b3SphereJoint;
	friend class b3ConeJoint;

	friend class b3List2<b3Body>;

	enum b3BodyFlags 
	{
		e_awakeFlag = 0x0001,
		e_islandFlag = 0x0002,
		e_fixedRotationX = 0x0004,
		e_fixedRotationY = 0x0008,
		e_fixedRotationZ = 0x0010,
	};

	b3Body(const b3BodyDef& def, b3World* world);
	~b3Body() { }

	// Destroy all shapes associated with the body.
	void DestroyShapes();

	// Destroy all contacts associated with the body.
	void DestroyContacts();

	// Destroy all joints connected to the body.
	void DestroyJoints();

	void SynchronizeTransform();
	void SynchronizeShapes();

	// Check if this body should collide with another.
	bool ShouldCollide(const b3Body* other) const;

	b3BodyType m_type;
	u32 m_islandID;
	u32 m_flags;
	float32 m_sleepTime;

	// The shapes attached to this body.
	b3List1<b3Shape> m_shapeList;
	
	// Joint edges for this body joint graph.
	b3List2<b3JointEdge> m_jointEdges;

	// User associated data (usually an entity).
	void* m_userData;

	// Body mass.
	float32 m_mass;

	// Inverse body mass.
	float32 m_invMass;
	
	// Inertia about the body local center of mass.
	b3Mat33 m_I;	
	
	// Inverse inertia about the body local center of mass.
	b3Mat33 m_invI;	
	
	// Inverse inertia about the body world center of mass.
	b3Mat33 m_worldInvI;
	
	b3Vec3 m_force;
	b3Vec3 m_torque;
	b3Vec3 m_linearVelocity;
	b3Vec3 m_angularVelocity;
	
	float32 m_linearDamping;
	float32 m_angularDamping;
	float32 m_gravityScale;
	
	// Motion proxy for CCD.
	b3Sweep m_sweep;

	// The body origin transform. 
	b3Transform m_xf;
		
	// The parent world of this body.
	b3World* m_world;
	
	// Links to the world body list.
	b3Body* m_prev;
	b3Body* m_next;
};

inline const b3Body* b3Body::GetNext() const
{
	return m_next;
}

inline b3Body* b3Body::GetNext()
{
	return m_next;
}

inline const b3World* b3Body::GetWorld() const 
{
	return m_world;
}

inline b3World* b3Body::GetWorld() 
{
	return m_world;
}

inline b3BodyType b3Body::GetType() const 
{ 
	return m_type; 
}

inline void* b3Body::GetUserData() const 
{ 
	return m_userData; 
}

inline void b3Body::SetUserData(void* userData) 
{ 
	m_userData = userData; 
}

inline const b3List1<b3Shape>& b3Body::GetShapeList() const
{
	return m_shapeList;
}

inline b3List1<b3Shape>& b3Body::GetShapeList()
{
	return m_shapeList;
}

inline const b3Transform& b3Body::GetTransform() const
{
	return m_xf;
}

inline void b3Body::SetTransform(const b3Vec3& position, const b3Vec3& axis, float32 angle) 
{
	b3Quat q = b3Quat(axis, angle);
	
	m_xf.position = position;
	m_xf.rotation = b3QuatMat33(q);

	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.orientation = q;

	m_sweep.worldCenter0 = m_sweep.worldCenter;
	m_sweep.orientation0 = m_sweep.orientation;

	SynchronizeShapes();
}

inline b3Vec3 b3Body::GetPosition() const
{
	return m_xf.position;
}

inline b3Quat b3Body::GetOrientation() const
{
	return m_sweep.orientation;
}

inline b3Vec3 b3Body::GetWorldCenter() const
{
	return m_sweep.worldCenter;
}

inline b3Vec3 b3Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}

inline b3Vec3 b3Body::GetLocalVector(const b3Vec3& vector) const
{
	return b3MulT(m_xf.rotation, vector);
}

inline b3Vec3 b3Body::GetWorldVector(const b3Vec3& localVector) const
{
	return b3Mul(m_xf.rotation, localVector);
}

inline b3Vec3 b3Body::GetLocalPoint(const b3Vec3& point) const
{
	return b3MulT(m_xf, point);
}

inline b3Vec3 b3Body::GetWorldPoint(const b3Vec3& point) const
{
	return b3Mul(m_xf, point);
}

inline b3Transform b3Body::GetLocalFrame(const b3Transform& xf) const
{
	return b3MulT(m_xf, xf);
}

inline b3Transform b3Body::GetWorldFrame(const b3Transform& xf) const
{
	return b3Mul(m_xf, xf);
}

inline const b3Sweep& b3Body::GetSweep() const
{
	return m_sweep;
}

inline bool b3Body::IsAwake() const
{
	return (m_flags & e_awakeFlag) != 0;
}

inline void b3Body::SetAwake(bool flag) 
{
	if (flag) 
	{
		if (!IsAwake()) 
		{
			m_flags |= e_awakeFlag;
			m_sleepTime = 0.0f;
		}
	}
	else 
	{
		m_flags &= ~e_awakeFlag;
		m_sleepTime = 0.0f;
		m_force.SetZero();
		m_torque.SetZero();
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();		
	}
}

inline float32 b3Body::GetLinearDamping() const
{
	return m_linearDamping;
}

inline void b3Body::SetLinearDamping(float32 damping) 
{
	m_linearDamping = damping;
}

inline float32 b3Body::GetAngularDamping() const
{
	return m_angularDamping;
}

inline void b3Body::SetAngularDamping(float32 damping) 
{
	m_angularDamping = damping;
}

inline float32 b3Body::GetGravityScale() const
{ 
	return m_gravityScale; 
}

inline void b3Body::SetGravityScale(float32 scale)
{
	if (m_type != e_staticBody) 
	{
		m_gravityScale = scale;
	}
}

inline b3Vec3 b3Body::GetPointVelocity(const b3Vec3& point) const
{
	return m_linearVelocity + b3Cross(m_angularVelocity, point - m_sweep.worldCenter);
}

inline b3Vec3 b3Body::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void b3Body::SetLinearVelocity(const b3Vec3& linearVelocity)
{
	if (m_type == e_staticBody) 
	{
		return;
	}
	
	if (b3Dot(linearVelocity, linearVelocity) > 0.0f) 
	{
		SetAwake(true);
	}

	m_linearVelocity = linearVelocity;
}

inline b3Vec3 b3Body::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline void b3Body::SetAngularVelocity(const b3Vec3& angularVelocity) 
{
	if (m_type == e_staticBody) 
	{
		return;
	}

	if (b3Dot(angularVelocity, angularVelocity) > 0.0f) 
	{
		SetAwake(true);
	}

	m_angularVelocity = angularVelocity;
}

inline float32 b3Body::GetMass() const
{
	return m_mass;
}

inline float32 b3Body::GetInverseMass() const
{
	return m_invMass;
}

inline const b3Mat33& b3Body::GetWorldInverseInertia() const
{
	return m_worldInvI;
}

inline const b3Mat33& b3Body::GetInertia() const
{
	return m_I;
}

inline float32 b3Body::GetLinearEnergy() const
{
	b3Vec3 P = m_mass * m_linearVelocity;
	return b3Dot(P, m_linearVelocity);
}

inline float32 b3Body::GetAngularEnergy() const
{
	b3Mat33 I = b3RotateToFrame(m_I, m_xf.rotation);
	b3Vec3 L = I * m_angularVelocity;
	return b3Dot(L, m_angularVelocity);
}

inline float32 b3Body::GetEnergy() const
{
	float32 e1 = GetLinearEnergy();
	float32 e2 = GetAngularEnergy();
	return 0.5f * (e1 + e2);
}

inline void b3Body::ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_force += force;
		m_torque += b3Cross(point - m_sweep.worldCenter, force);
	}
}

inline void b3Body::ApplyForceToCenter(const b3Vec3& force, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_force += force;
	}
}

inline void b3Body::ApplyTorque(const b3Vec3& torque, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_torque += torque;
	}
}

inline void b3Body::ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& worldPoint, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += b3Mul(m_worldInvI, b3Cross(worldPoint - m_sweep.worldCenter, impulse));
	}
}

inline void b3Body::ApplyAngularImpulse(const b3Vec3& impulse, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_angularVelocity += b3Mul(m_worldInvI, impulse);
	}
}

#endif
