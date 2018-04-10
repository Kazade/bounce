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

#ifndef B3_BOX_HULL_H
#define B3_BOX_HULL_H

#include <bounce/collision/shapes/hull.h>

struct b3BoxHull : public b3Hull
{
	b3Vec3 boxVertices[8];
	b3HalfEdge boxEdges[24];
	b3Face boxFaces[6];
	b3Plane boxPlanes[6];

	// Does nothing for performance.
	b3BoxHull() { }

	// Construct this box from three extents and centered at the origin.
	b3BoxHull(float32 ex, float32 ey, float32 ez)
	{
		Set(ex, ey, ez);
	}
		
	// Set this box to the unit box centered at the origin.
	void SetIdentity()
	{
		boxVertices[0] = b3Vec3(1.0f, 1.0f, -1.0f);
		boxVertices[1] = b3Vec3(-1.0f, 1.0f, -1.0f);
		boxVertices[2] = b3Vec3(-1.0f, -1.0f, -1.0f);
		boxVertices[3] = b3Vec3(1.0f, -1.0f, -1.0f);
		boxVertices[4] = b3Vec3(1.0f, 1.0f, 1.0f);
		boxVertices[5] = b3Vec3(-1.0f, 1.0f, 1.0f);
		boxVertices[6] = b3Vec3(-1.0f, -1.0f, 1.0f);
		boxVertices[7] = b3Vec3(1.0f, -1.0f, 1.0f);

		boxEdges[0] = b3MakeEdge(1, 1, 0, 2);
		boxEdges[1] = b3MakeEdge(2, 0, 5, 21);
		boxEdges[2] = b3MakeEdge(2, 3, 0, 4);
		boxEdges[3] = b3MakeEdge(6, 2, 2, 18);
		boxEdges[4] = b3MakeEdge(6, 5, 0, 6);
		boxEdges[5] = b3MakeEdge(5, 4, 4, 17);
		boxEdges[6] = b3MakeEdge(5, 7, 0, 0);
		boxEdges[7] = b3MakeEdge(1, 6, 3, 22);
		boxEdges[8] = b3MakeEdge(4, 9, 1, 10);
		boxEdges[9] = b3MakeEdge(7, 8, 4, 23);
		boxEdges[10] = b3MakeEdge(7, 11, 1, 12);
		boxEdges[11] = b3MakeEdge(3, 10, 2, 16);
		boxEdges[12] = b3MakeEdge(3, 13, 1, 14);
		boxEdges[13] = b3MakeEdge(0, 12, 5, 19);
		boxEdges[14] = b3MakeEdge(0, 15, 1, 8);
		boxEdges[15] = b3MakeEdge(4, 14, 3, 20);
		boxEdges[16] = b3MakeEdge(7, 17, 2, 3);
		boxEdges[17] = b3MakeEdge(6, 16, 4, 9);
		boxEdges[18] = b3MakeEdge(2, 19, 2, 11);
		boxEdges[19] = b3MakeEdge(3, 18, 5, 1);
		boxEdges[20] = b3MakeEdge(0, 21, 3, 7);
		boxEdges[21] = b3MakeEdge(1, 20, 5, 13);
		boxEdges[22] = b3MakeEdge(5, 23, 3, 15);
		boxEdges[23] = b3MakeEdge(4, 22, 4, 5);

		boxFaces[0].edge = 6;
		boxFaces[1].edge = 14;
		boxFaces[2].edge = 18;
		boxFaces[3].edge = 15;
		boxFaces[4].edge = 9;
		boxFaces[5].edge = 21;

		boxPlanes[0] = b3Plane(b3Vec3(-1.0f, 0.0f, 0.0f), boxVertices[1]);
		boxPlanes[1] = b3Plane(b3Vec3(1.0f, 0.0f, 0.0f), boxVertices[0]);
		boxPlanes[2] = b3Plane(b3Vec3(0.0f, -1.0f, 0.0f), boxVertices[2]);
		boxPlanes[3] = b3Plane(b3Vec3(0.0f, 1.0f, 0.0f), boxVertices[1]);
		boxPlanes[4] = b3Plane(b3Vec3(0.0f, 0.0f, 1.0f), boxVertices[4]);
		boxPlanes[5] = b3Plane(b3Vec3(0.0f, 0.0f, -1.0f), boxVertices[0]);

		centroid = b3Vec3(0.0f, 0.0f, 0.0f);
		vertices = boxVertices;
		vertexCount = 8;
		edges = boxEdges;
		edgeCount = 24;
		faces = boxFaces;
		planes = boxPlanes;
		faceCount = 6;
		
		Validate();
	}
	
	// Set this box from three extents and centered at the origin.
	void Set(float32 ex, float32 ey, float32 ez)
	{
		b3Transform xf;
		xf.position.SetZero();
		xf.rotation = b3Diagonal(ex, ey, ez);
		SetTransform(xf);
	}

	// Set this box to the unit box and transform it. 
	void SetTransform(const b3Transform& T)
	{
		boxVertices[0] = b3Vec3(1.0f, 1.0f, -1.0f);
		boxVertices[1] = b3Vec3(-1.0f, 1.0f, -1.0f);
		boxVertices[2] = b3Vec3(-1.0f, -1.0f, -1.0f);
		boxVertices[3] = b3Vec3(1.0f, -1.0f, -1.0f);
		boxVertices[4] = b3Vec3(1.0f, 1.0f, 1.0f);
		boxVertices[5] = b3Vec3(-1.0f, 1.0f, 1.0f);
		boxVertices[6] = b3Vec3(-1.0f, -1.0f, 1.0f);
		boxVertices[7] = b3Vec3(1.0f, -1.0f, 1.0f);

		for (u32 i = 0; i < 8; ++i)
		{
			boxVertices[i] = T * boxVertices[i];
		}

		boxEdges[0] = b3MakeEdge(1, 1, 0, 2);
		boxEdges[1] = b3MakeEdge(2, 0, 5, 21);
		boxEdges[2] = b3MakeEdge(2, 3, 0, 4);
		boxEdges[3] = b3MakeEdge(6, 2, 2, 18);
		boxEdges[4] = b3MakeEdge(6, 5, 0, 6);
		boxEdges[5] = b3MakeEdge(5, 4, 4, 17);
		boxEdges[6] = b3MakeEdge(5, 7, 0, 0);
		boxEdges[7] = b3MakeEdge(1, 6, 3, 22);
		boxEdges[8] = b3MakeEdge(4, 9, 1, 10);
		boxEdges[9] = b3MakeEdge(7, 8, 4, 23);
		boxEdges[10] = b3MakeEdge(7, 11, 1, 12);
		boxEdges[11] = b3MakeEdge(3, 10, 2, 16);
		boxEdges[12] = b3MakeEdge(3, 13, 1, 14);
		boxEdges[13] = b3MakeEdge(0, 12, 5, 19);
		boxEdges[14] = b3MakeEdge(0, 15, 1, 8);
		boxEdges[15] = b3MakeEdge(4, 14, 3, 20);
		boxEdges[16] = b3MakeEdge(7, 17, 2, 3);
		boxEdges[17] = b3MakeEdge(6, 16, 4, 9);
		boxEdges[18] = b3MakeEdge(2, 19, 2, 11);
		boxEdges[19] = b3MakeEdge(3, 18, 5, 1);
		boxEdges[20] = b3MakeEdge(0, 21, 3, 7);
		boxEdges[21] = b3MakeEdge(1, 20, 5, 13);
		boxEdges[22] = b3MakeEdge(5, 23, 3, 15);
		boxEdges[23] = b3MakeEdge(4, 22, 4, 5);

		boxFaces[0].edge = 6;
		boxFaces[1].edge = 14;
		boxFaces[2].edge = 18;
		boxFaces[3].edge = 15;
		boxFaces[4].edge = 9;
		boxFaces[5].edge = 21;

		boxPlanes[0] = b3Plane(b3Vec3(-1.0f, 0.0f, 0.0f), boxVertices[1]);
		boxPlanes[1] = b3Plane(b3Vec3(1.0f, 0.0f, 0.0f), boxVertices[0]);
		boxPlanes[2] = b3Plane(b3Vec3(0.0f, -1.0f, 0.0f), boxVertices[2]);
		boxPlanes[3] = b3Plane(b3Vec3(0.0f, 1.0f, 0.0f), boxVertices[1]);
		boxPlanes[4] = b3Plane(b3Vec3(0.0f, 0.0f, 1.0f), boxVertices[4]);
		boxPlanes[5] = b3Plane(b3Vec3(0.0f, 0.0f, -1.0f), boxVertices[0]);

		centroid = b3Vec3(0.0f, 0.0f, 0.0f);
		vertices = boxVertices;
		vertexCount = 8;
		edges = boxEdges;
		edgeCount = 24;
		faces = boxFaces;
		planes = boxPlanes;
		faceCount = 6;

		centroid = T * centroid;

		Validate();
	}
};

extern const b3BoxHull b3BoxHull_identity;

#endif