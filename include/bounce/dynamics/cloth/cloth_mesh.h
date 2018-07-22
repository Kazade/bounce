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

#ifndef B3_CLOTH_MESH_H
#define B3_CLOTH_MESH_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/vec3.h>

struct b3ClothMeshTriangle
{
	u32 v1, v2, v3;
};

struct b3ClothMeshMesh
{
	u32 vertexCount;
	u32 startVertex;
	u32 triangleCount;
	u32 startTriangle;
};

struct b3ClothMeshSewingLine
{
	u32 s1, s2;
	u32 v1, v2;
};

struct b3ClothMesh
{
	u32 vertexCount;
	b3Vec3* vertices;
	u32 triangleCount;
	b3ClothMeshTriangle* triangles;
	u32 meshCount;
	b3ClothMeshMesh* meshes;
	u32 sewingLineCount;
	b3ClothMeshSewingLine* sewingLines;
};

struct b3GarmentMesh;

// Convenience structure.
struct b3GarmentClothMesh : public b3ClothMesh
{
	b3GarmentClothMesh();
	~b3GarmentClothMesh();

	// Set this mesh from a 2D garment mesh.
	void Set(const b3GarmentMesh* garment);
};

#endif