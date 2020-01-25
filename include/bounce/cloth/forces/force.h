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

#ifndef B3_FORCE_H
#define B3_FORCE_H

#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>

class b3ClothParticle;

struct b3ClothForceSolverData;

// Force types
enum b3ForceType
{
	e_stretchForce,
	e_shearForce,
	e_springForce,
	e_mouseForce,
	e_elementForce,
};

struct b3ForceDef
{
	b3ForceType type;
};

// A force acts on a set of particles.
class b3Force
{
public:
	// Get the force type.
	b3ForceType GetType() const;

	// Has this force a given particle?
	virtual bool HasParticle(const b3ClothParticle* particle) const = 0;

	// Get the next force in the cloth force list.
	const b3Force* GetNext() const;
	b3Force* GetNext();
protected:
	friend class b3List2<b3Force>;
	friend class b3Cloth;
	friend class b3ClothParticle;
	friend class b3ClothForceSolver;

	// Factory create and destroy.
	static b3Force* Create(const b3ForceDef* def);
	static void Destroy(b3Force* f);

	b3Force() { }
	virtual ~b3Force() { }

	virtual void Apply(const b3ClothForceSolverData* data) = 0;

	b3ForceType m_type;

	b3Force* m_prev;
	b3Force* m_next;
};

inline b3ForceType b3Force::GetType() const
{
	return m_type;
}

inline const b3Force* b3Force::GetNext() const
{
	return m_next;
}

inline b3Force* b3Force::GetNext()
{
	return m_next;
}

#endif