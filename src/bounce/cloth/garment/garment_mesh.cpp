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

#include <bounce/cloth/garment/garment_mesh.h>
#include <bounce/cloth/garment/garment.h>
#include <bounce/cloth/garment/sewing_pattern.h>

#define ANSI_DECLARATORS
#define REAL double
#define VOID void

extern "C"
{
	#include <triangle/triangle.h>
}

b3GarmentMesh::b3GarmentMesh()
{
	meshCount = 0;
	meshes = nullptr;
	sewingCount = 0;
	sewingLines = nullptr;
	garment = nullptr;
}

b3GarmentMesh::~b3GarmentMesh()
{
	for (u32 i = 0; i < meshCount; ++i)
	{
		b3Free(meshes[i].vertices);
		b3Free(meshes[i].triangles);
	}
	
	b3Free(meshes);
	b3Free(sewingLines);
}

// 
static void b3Set(b3SewingPatternMesh* mesh, float32 desiredArea, const b3SewingPattern* pattern)
{
	B3_ASSERT(desiredArea > B3_EPSILON);
	
	struct triangulateio in, mid, out;

	// Prepare the input structure
	in.pointlist = (REAL*)malloc(pattern->vertexCount * 2 * sizeof(REAL));
	const float32* fp = (float32*)pattern->vertices;
	for (u32 i = 0; i < 2 * pattern->vertexCount; ++i)
	{
		in.pointlist[i] = (REAL)fp[i];
	}
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofpoints = pattern->vertexCount;
	in.numberofpointattributes = 0;

	in.trianglelist = NULL;
	in.triangleattributelist = NULL;

	in.trianglearealist = NULL;

	in.numberoftriangles = 0;
	in.numberofcorners = 0;
	in.numberoftriangleattributes = 0;

	in.segmentlist = NULL;
	in.segmentmarkerlist = NULL;
	in.numberofsegments = 0;

	in.holelist = NULL;
	in.numberofholes = 0;

	in.regionlist = NULL;
	in.numberofregions = 0;

	// Prepare the middle structure
	mid.pointlist = NULL;
	mid.pointmarkerlist = NULL;

	mid.trianglelist = NULL;
	mid.triangleattributelist = NULL;
	mid.trianglearealist = NULL;
	mid.neighborlist = NULL;

	mid.segmentlist = NULL;
	mid.segmentmarkerlist = NULL;

	// Run triangulation
	// z - zero based indices
	// p - PSLG
	// c - preserve the convex hull
	triangulate("zpc", &in, &mid, NULL);

	// Refine

	// Prepare middle structure
	mid.trianglearealist = (REAL*)malloc(mid.numberoftriangles * sizeof(REAL));
	for (int i = 0; i < mid.numberoftriangles; ++i)
	{
		mid.trianglearealist[i] = desiredArea;
	}

	// Prepare output structure
	out.pointlist = NULL;
	out.pointmarkerlist = NULL;

	out.trianglelist = NULL;
	out.trianglearealist = NULL;

	out.segmentlist = NULL;
	out.segmentmarkerlist = NULL;

	// Run triangulation
	// z - zero based indices
	// p - PSLG
	// c - preserve the convex hull
	// r - read triangles
	triangulate("zpcra", &mid, &out, NULL);

	// Convert the output structure 

	// The mesh must be empty
	mesh->vertices = (b3Vec2*)b3Alloc(out.numberofpoints * sizeof(b3Vec2));
	mesh->vertexCount = out.numberofpoints;
	for (int i = 0; i < out.numberofpoints; ++i)
	{
		mesh->vertices[i].x = (float32)out.pointlist[2 * i + 0];
		mesh->vertices[i].y = (float32)out.pointlist[2 * i + 1];
	}

	mesh->triangles = (b3SewingPatternMeshTriangle*)b3Alloc(out.numberoftriangles * sizeof(b3SewingPatternMeshTriangle));
	mesh->triangleCount = out.numberoftriangles;
	for (int i = 0; i < out.numberoftriangles; ++i)
	{
		B3_ASSERT(out.numberofcorners == 3);

		b3SewingPatternMeshTriangle triangle;
		triangle.v1 = out.trianglelist[3 * i + 0];
		triangle.v2 = out.trianglelist[3 * i + 1];
		triangle.v3 = out.trianglelist[3 * i + 2];

		mesh->triangles[i] = triangle;
	}

	// Free the input structure
	free(in.pointlist);

	// Free the middle structure
	free(mid.pointlist);
	free(mid.pointmarkerlist);
	free(mid.trianglelist);
	free(mid.triangleattributelist);
	free(mid.trianglearealist);
	free(mid.segmentlist);
	free(mid.segmentmarkerlist);

	// Free the output structure
	free(out.pointlist);
	free(out.pointmarkerlist);
	free(out.trianglelist);
	free(out.segmentlist);
	free(out.segmentmarkerlist);
}

void b3GarmentMesh::Set(b3Garment* g, float32 desiredArea)
{
	garment = g;
	meshCount = garment->patternCount;
	meshes = (b3SewingPatternMesh*)b3Alloc(garment->patternCount * sizeof(b3SewingPatternMesh));
	for (u32 i = 0; i < garment->patternCount; ++i)
	{
		b3Set(meshes + i, desiredArea, garment->patterns[i]);
	}
}