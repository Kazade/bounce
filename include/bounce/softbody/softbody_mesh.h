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

#ifndef B3_SOFT_BODY_MESH_H
#define B3_SOFT_BODY_MESH_H

#include <bounce/common/math/vec3.h>

struct b3SoftBodyMeshTetrahedron
{
	u32 v1, v2, v3, v4;
};

struct b3SoftBodyMesh
{
	u32 vertexCount;
	b3Vec3* vertices;
	u32 tetrahedronCount;
	b3SoftBodyMeshTetrahedron* tetrahedrons;
};

struct b3QSoftBodyMesh : public b3SoftBodyMesh
{
	b3QSoftBodyMesh();
	~b3QSoftBodyMesh();

	void SetAsSphere(float32 radius, u32 subdivisions);
	
	void SetAsCylinder(float32 radius, float32 ey, u32 segments);
};

#endif