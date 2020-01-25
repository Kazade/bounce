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

#ifndef B3_CLOTH_WORLD_SHAPE_H
#define B3_CLOTH_WORLD_SHAPE_H

#include <bounce/cloth/shapes/cloth_shape.h>
#include <bounce/common/template/list.h>
#include <bounce/collision/shapes/aabb.h>

class b3Cloth;
class b3Shape;

struct b3ClothWorldShapeDef : b3ClothShapeDef
{
	const b3Shape* shape;
};

class b3ClothWorldShape : public b3ClothShape
{
public:
	// Return the attached world shape.
	const b3Shape* GetShape() { return m_shape; }
	const b3Shape* GetShape() const { return m_shape; }

	// Return the next world shape in the cloth list of world shapes.
	b3ClothWorldShape* GetNext() { return m_next; }
	const b3ClothWorldShape* GetNext() const { return m_next; }
private:
	friend class b3Cloth;
	friend class b3ClothContactManager;
	friend class b3ClothSphereAndShapeContact;
	friend class b3ClothContactSolver;
	friend class b3List2<b3ClothWorldShape>;

	b3ClothWorldShape(const b3ClothWorldShapeDef& def, b3Cloth* cloth);
	~b3ClothWorldShape();

	// Compute AABB.
	b3AABB ComputeAABB() const;

	// Synchronize AABB.
	void Synchronize(const b3Vec3& displacement);

	// Destroy contacts.
	void DestroyContacts();

	// Shape in the world.
	const b3Shape* m_shape;
	
	// Cloth list links.
	b3ClothWorldShape* m_prev;
	b3ClothWorldShape* m_next;
};

#endif