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

#ifndef B3_CONTACT_CLUSTER_H
#define B3_CONTACT_CLUSTER_H

#include <bounce/common/geometry.h>
#include <bounce/common/template/array.h>
#include <bounce/dynamics/contacts/manifold.h>

#define B3_NULL_CLUSTER (-1)

// Used for contact cluster reduction.
struct b3ClusterVertex
{
	b3Vec3 position; // point on the cluster plane
	u32 clipIndex; // where did this vertex came from (hint: local point)
};

// Used for contact cluster reduction.
typedef b3Array<b3ClusterVertex> b3ClusterPolygon;

// A observation represents a contact normal.	
struct b3Observation
{
	u32 manifold;
	u32 manifoldPoint;
	b3Vec3 point; // normal
	i32 cluster; // normal
};

// A group of contact points with a similar contact normal.
struct b3Cluster
{
	b3Vec3 centroid;
};

// Initialize a set of clusters.
void b3InitializeClusters(b3Array<b3Cluster>& outClusters, 
const b3Array<b3Observation>& inObservations);

// Run the cluster algorithm.
void b3Clusterize(b3Array<b3Cluster>& outClusters, b3Array<b3Observation>& outObservations,
	const b3Array<b3Cluster>& inClusters, const b3Array<b3Observation>& inObservations);

u32 b3Clusterize(b3Manifold outManifolds[3], const b3Manifold* inManifolds, u32 numIn,
	const b3Transform& xfA, float32 radiusA, const b3Transform& xfB, float32 radiusB);

// Reduce a set of contact points to a quad (approximate convex polygon).
// All points must lie in a common plane and an initial point must be given.
void b3ReducePolygon(b3ClusterPolygon& pOut, const b3ClusterPolygon& pIn, 
	u32 startIndex, const b3Vec3& normal);

#endif
