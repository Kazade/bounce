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

#include <bounce/cloth/cloth_mesh.h>
#include <bounce/cloth/garment/garment.h>
#include <bounce/cloth/garment/garment_mesh.h>
#include <bounce/cloth/garment/sewing_pattern.h>

b3GarmentClothMesh::b3GarmentClothMesh()
{
	vertexCount = 0;
	vertices = nullptr;
	triangleCount = 0;
	triangles = nullptr;
	meshCount = 0;
	meshes = nullptr;
	sewingLineCount = 0;
	sewingLines = nullptr;
}

b3GarmentClothMesh::~b3GarmentClothMesh()
{
	b3Free(vertices);
	b3Free(triangles);
	b3Free(meshes);
	b3Free(sewingLines);
}

void b3GarmentClothMesh::Set(const b3GarmentMesh* garment)
{
	B3_ASSERT(vertexCount == 0);
	for (u32 i = 0; i < garment->meshCount; ++i)
	{
		vertexCount += garment->meshes[i].vertexCount;
	}
	vertices = (b3Vec3*)b3Alloc(vertexCount * sizeof(b3Vec3));
	
	B3_ASSERT(triangleCount == 0);
	for (u32 i = 0; i < garment->meshCount; ++i)
	{
		triangleCount += garment->meshes[i].triangleCount;
	}
	triangles = (b3ClothMeshTriangle*)b3Alloc(triangleCount * sizeof(b3ClothMeshTriangle));

	B3_ASSERT(meshCount == 0);
	meshCount = garment->meshCount;
	meshes = (b3ClothMeshMesh*)b3Alloc(meshCount * sizeof(b3ClothMeshMesh));

	B3_ASSERT(sewingLineCount == 0);
	sewingLineCount = garment->sewingCount;
	sewingLines = (b3ClothMeshSewingLine*)b3Alloc(sewingLineCount * sizeof(b3ClothMeshSewingLine));

	u32 vertex_count = 0;
	for (u32 pattern_index = 0; pattern_index < garment->meshCount; ++pattern_index)
	{
		u32 pattern_vertex_count = garment->meshes[pattern_index].vertexCount;

		meshes[pattern_index].vertexCount = pattern_vertex_count;
		meshes[pattern_index].startVertex = vertex_count;

		for (u32 pattern_vertex = 0; pattern_vertex < pattern_vertex_count; ++pattern_vertex)
		{
			vertices[vertex_count].x = garment->meshes[pattern_index].vertices[pattern_vertex].x;
			vertices[vertex_count].y = garment->meshes[pattern_index].vertices[pattern_vertex].y;
			vertices[vertex_count].z = 0.0f;
			
			++vertex_count;
		}
	}

	u32 triangle_count = 0;
	for (u32 pattern_index = 0; pattern_index < garment->meshCount; ++pattern_index)
	{
		u32 pattern_triangle_count = garment->meshes[pattern_index].triangleCount;

		meshes[pattern_index].triangleCount = pattern_triangle_count;
		meshes[pattern_index].startTriangle = triangle_count;

		for (u32 pattern_triangle = 0; pattern_triangle < pattern_triangle_count; ++pattern_triangle)
		{
			u32 pattern_v1 = garment->meshes[pattern_index].triangles[pattern_triangle].v1;
			u32 pattern_v2 = garment->meshes[pattern_index].triangles[pattern_triangle].v2;
			u32 pattern_v3 = garment->meshes[pattern_index].triangles[pattern_triangle].v3;

			u32 cloth_v1 = meshes[pattern_index].startVertex + pattern_v1;
			u32 cloth_v2 = meshes[pattern_index].startVertex + pattern_v2;
			u32 cloth_v3 = meshes[pattern_index].startVertex + pattern_v3;

			triangles[triangle_count].v1 = cloth_v1;
			triangles[triangle_count].v2 = cloth_v2;
			triangles[triangle_count].v3 = cloth_v3;

			++triangle_count;
		}
	}

	for (u32 sewing_line_index = 0; sewing_line_index < garment->sewingCount; ++sewing_line_index)
	{
		sewingLines[sewing_line_index].s1 = garment->sewingLines[sewing_line_index].s1;
		sewingLines[sewing_line_index].v1 = meshes[sewingLines[sewing_line_index].s1].startVertex + garment->sewingLines[sewing_line_index].v1;

		sewingLines[sewing_line_index].s2 = garment->sewingLines[sewing_line_index].s2;
		sewingLines[sewing_line_index].v2 = meshes[sewingLines[sewing_line_index].s2].startVertex + garment->sewingLines[sewing_line_index].v2;
	}
}