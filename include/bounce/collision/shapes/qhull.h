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

#ifndef B3_Q_HULL_H
#define B3_Q_HULL_H

#include <bounce/collision/shapes/hull.h>
#include <bounce/common/template/array.h>

// This hull can be constructed from an array of points.
struct b3QHull : public b3Hull
{
	b3StackArray<b3Vec3, 256> hullVertices;
	b3StackArray<b3HalfEdge, 256> hullEdges;
	b3StackArray<b3Face, 256> hullFaces;
	b3StackArray<b3Plane, 256> hullPlanes;

	b3QHull()
	{
		vertices = hullVertices.Begin();
		vertexCount = 0;
		edges = hullEdges.Begin();
		edgeCount = 0;
		faces = hullFaces.Begin();
		faceCount = 0;
		planes = hullPlanes.Begin();
		centroid.SetZero();
	}

	// Create a convex hull from vertex data.
	// If the creation has failed then this convex hull is not modified.
	// vertexStride - size of bytes between vertices
	// vertexBase - pointer to the first vertex
	// vertexCount - number of vertices
	// simplify - if set to true the convex hull is simplified after initial construction
	void Set(u32 vertexStride, const void* vertexBase, u32 vertexCount, bool simplify = true);

	// Set this hull as a sphere located at the origin
	// given the radius.
	void SetAsSphere(float32 radius = 1.0f);

	// Set this hull as a cylinder located at the origin
	// given the radius and extent along the y axis.
	void SetAsCylinder(float32 radius = 1.0f, float32 ey = 1.0f);

	// Set this hull as a cone located at the origin 
	// given the radius and extent along the y axis.
	void SetAsCone(float32 radius = 1.0f, float32 ey = 1.0f);
};

#endif