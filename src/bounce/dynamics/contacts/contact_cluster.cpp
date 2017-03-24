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

#include <bounce/dynamics/contacts/contact_cluster.h>
#include <bounce/collision/collision.h>

static void AddCluster(b3Array<b3Cluster>& clusters, const b3Vec3& centroid)
{
	const float32 kTol = 0.05f;
	for (u32 i = 0; i < clusters.Count(); ++i)
	{
		b3Vec3 c = clusters[i].centroid;
		float32 dd = b3DistanceSquared(centroid, c);
		if (dd < kTol * kTol)
		{
			// Merge the clusters.
			clusters[i].centroid += centroid;
			clusters[i].centroid.Normalize();
			return;
		}
	}

	b3Cluster c;
	c.centroid = centroid;
	clusters.PushBack(c);
}

void b3InitializeClusters(b3Array<b3Cluster>& outClusters, const b3Array<b3Observation>& inObs)
{
	B3_ASSERT(outClusters.IsEmpty());
	
	const u32 kMaxClusters = 3;

	if (inObs.Count() <= kMaxClusters)
	{
		for (u32 i = 0; i < inObs.Count(); ++i)
		{
			const b3Observation& o = inObs[i];
			AddCluster(outClusters, o.point);
		}
		return;
	}

	B3_ASSERT(inObs.Count() > 3);

	// This is used to skip observations that were 
	// used to initialize a cluster centroid.
	b3StackArray<bool, 64> chosens;
	chosens.Resize(inObs.Count());
	for (u32 i = 0; i < inObs.Count(); ++i)
	{
		chosens[i] = false;
	}
	
	// Choose the most opposing faces.
	{
		u32 index = 0;
		const b3Observation& o = inObs[index];
		b3Vec3 A = o.point;
		AddCluster(outClusters, A);
		chosens[index] = true;
	}

	{
		b3Vec3 A = outClusters[0].centroid;

		u32 index = 0;
		float32 max = -B3_MAX_FLOAT;
		for (u32 i = 0; i < inObs.Count(); ++i)
		{
			if (chosens[i])	{ continue;	}

			const b3Observation& o = inObs[i];

			b3Vec3 B = o.point;
			float32 dd = b3DistanceSquared(A, B);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = inObs[index];
		b3Vec3 B = o.point;
		AddCluster(outClusters, B);
		chosens[index] = true;
	}

	B3_ASSERT(outClusters.Count() == 1 || outClusters.Count() == 2);

	if (outClusters.Count() == 1)
	{
		b3Vec3 A = outClusters[0].centroid;

		u32 index = 0;
		float32 max = -B3_MAX_FLOAT;
		for (u32 i = 0; i < inObs.Count(); ++i)
		{
			if (chosens[i])	{ continue; }

			const b3Observation& o = inObs[i];
			
			b3Vec3 B = o.point;

			float32 dd = b3DistanceSquared(A, B);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = inObs[index];
		b3Vec3 B = o.point;
		AddCluster(outClusters, B);
		chosens[index] = true;
		return;
	}

	B3_ASSERT(outClusters.Count() == 2);

	{
		b3Vec3 A = outClusters[0].centroid;
		b3Vec3 B = outClusters[1].centroid;

		u32 index = 0;
		float32 max = -B3_MAX_FLOAT;
		for (u32 i = 0; i < inObs.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			const b3Observation& o = inObs[i];
			
			b3Vec3 C = o.point;
			b3Vec3 Q = b3ClosestPointOnSegment(C, A, B);

			float32 dd = b3DistanceSquared(C, Q);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = inObs[index];
		b3Vec3 C = o.point;
		AddCluster(outClusters, C);
		chosens[index] = true;
	}
}

static void b3MoveObsToCluster(b3Array<b3Observation>& observations, u32 fromCluster, u32 toCluster)
{
	for (u32 i = 0; i < observations.Count(); ++i)
	{
		b3Observation& obs = observations[i];
		if (obs.cluster == fromCluster)
		{
			obs.cluster = toCluster;
		}
	}
}

static u32 b3BestCluster(const b3Array<b3Cluster>& clusters, const b3Vec3& point)
{
	u32 bestIndex = 0;
	float32 bestValue = B3_MAX_FLOAT;
	for (u32 i = 0; i < clusters.Count(); ++i)
	{
		b3Vec3 centroid = clusters[i].centroid;
		float32 metric = b3DistanceSquared(point, centroid);
		if (metric < bestValue)
		{
			bestValue = metric;
			bestIndex = i;
		}
	}
	return bestIndex;
}

void b3Clusterize(b3Array<b3Cluster>& outClusters, b3Array<b3Observation>& outObservations,
	const b3Array<b3Cluster>& inClusters, const b3Array<b3Observation>& inObservations)
{
	B3_ASSERT(outObservations.IsEmpty());
	B3_ASSERT(outClusters.IsEmpty());

	// Temporary data
	b3StackArray<b3Cluster, 32> clusters;
	clusters.Swap(inClusters);
	
	b3StackArray<b3Observation, 32> observations;
	observations.Swap(inObservations);

	// Termination criteria for k-means clustering 
	const u32 kMaxIters = 10;

	u32 iter = 0;
	while (iter < kMaxIters)
	{
		// Assign each observation to the closest cluster centroid.
		for (u32 i = 0; i < observations.Count(); ++i)
		{
			b3Observation& obs = observations[i];
			obs.cluster = b3BestCluster(clusters, obs.point);
		}

		// Compute the new cluster centroids.
		for (u32 i = 0; i < clusters.Count(); ++i)
		{
			b3Cluster& cluster = clusters[i];
			
			b3Vec3 centroid;
			centroid.SetZero();

			u32 pointCount = 0;
			for (u32 j = 0; j < observations.Count(); ++j)
			{
				const b3Observation& obs = observations[j];
				if (obs.cluster == i)
				{
					centroid += obs.point;
					++pointCount;
				}
			}

			if (pointCount > 0)
			{
				centroid /= float32(pointCount);
				cluster.centroid = centroid;
			}
		}

		++iter;
	}

	// Remove empty clusters.
	outObservations.Swap(observations);
	
	u32 numOut = 0;

	for (u32 i = 0; i < clusters.Count(); ++i)
	{
		u32 pointCount = 0;
		for (u32 j = 0; j < outObservations.Count(); ++j)
		{
			const b3Observation& obs = outObservations[j];
			if (obs.cluster == i)
			{
				++pointCount;
			}
		}

		if (pointCount > 0)
		{
			// Transfer the clusters observations into the newly cluster.
			b3MoveObsToCluster(outObservations, i, numOut);
			outClusters.PushBack(clusters[i]);
			++numOut;
		}
	}
}

static B3_FORCE_INLINE bool b3IsCCW(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& N)
{
	b3Vec3 n = b3Cross(B - A, C - A);
	return b3Dot(n, N) > 0.0f;
}

static B3_FORCE_INLINE bool b3IsCCW(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D, const b3Vec3& N)
{
	return b3IsCCW(A, B, C, N) && b3IsCCW(C, D, A, N);
}

static void b3Weld(b3ClusterPolygon& pOut, const b3Vec3& N)
{
	B3_ASSERT(pOut.Count() > 0);
	b3Vec3 A = pOut[0].position;
	for (u32 i = 1; i < pOut.Count(); ++i)
	{
		b3ClusterVertex& vi = pOut[i];
		b3Vec3 B = vi.position;
		for (u32 j = i + 1; j < pOut.Count(); ++j)
		{
			b3ClusterVertex& vj = pOut[j];
			b3Vec3 C = vj.position;
			if (b3IsCCW(A, B, C, N) == false)
			{
				b3Swap(vi, vj);
			}
		}
	}
}

void b3ReducePolygon(b3ClusterPolygon& pOut,
	const b3ClusterPolygon& pIn, u32 startIndex, const b3Vec3& normal)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);
	B3_ASSERT(startIndex < pIn.Count());
		
	pOut.Reserve(pIn.Count());
	if (pIn.Count() <= B3_MAX_MANIFOLD_POINTS)
	{
		pOut = pIn;
		b3Weld(pOut, normal);
		return;
	}

	B3_ASSERT(pIn.Count() > B3_MAX_MANIFOLD_POINTS);
	
	b3StackArray<bool, 32> chosens;
	chosens.Resize(pIn.Count());
	for (u32 i = 0; i < chosens.Count(); ++i)
	{
		chosens[i] = false;
	}

	{
		u32 index = startIndex;
		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;

		u32 index = 0;
		float32 max = -B3_MAX_FLOAT;
		for (u32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 B = pIn[i].position;
			b3Vec3 d = B - A;
			float32 dd = b3Dot(d, d);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		// Coincidence check.
		if (max < B3_EPSILON * B3_EPSILON)
		{
			return;
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;
		b3Vec3 B = pOut[1].position;

		u32 index = 0;
		float32 max = -B3_MAX_FLOAT;
		for (u32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 C = pIn[i].position;
			b3Vec3 N = b3Cross(B - A, C - A);
			float32 sa2 = b3Dot(N, normal);
			float32 a2 = b3Abs(sa2);
			if (a2 > max)
			{
				max = a2;
				index = i;
			}
		}

		// Colinearity check.
		// Use wanky tolerance for reasonable performance.
		const float32 kAreaTol = 0.01f;
		if (max < 2.0f * kAreaTol)
		{
			// Return the largest segment AB.
			return;
		}

		// Ensure CCW order of triangle ABC.
		b3Vec3 C = pIn[index].position;
		if (b3IsCCW(A, B, C, normal) == false)
		{
			b3Swap(pOut[0], pOut[1]);
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;
		b3Vec3 B = pOut[1].position;
		b3Vec3 C = pOut[2].position;

		B3_ASSERT(b3IsCCW(A, B, C, normal));

		u32 index = 0;
		float32 min = B3_MAX_FLOAT;
		for (u32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 D = pIn[i].position;
			b3Vec3 N = b3Cross(B - A, D - A);
			float32 sa2 = b3Dot(N, normal);
			if (sa2 < min)
			{
				min = sa2;
				index = i;
			}
		}

		// Colinearity check.
		const float32 kAreaTol = 0.01f;
		if (b3Abs(min) < 2.0f * kAreaTol)
		{
			// Return the face ABC.
			return;
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	B3_ASSERT(pOut.Count() <= B3_MAX_MANIFOLD_POINTS);
	b3Weld(pOut, normal);
}

u32 b3Clusterize(b3Manifold outManifolds[3], const b3Manifold* inManifolds, u32 numIn,
	const b3Transform& xfA, float32 radiusA, const b3Transform& xfB, float32 radiusB)
{
	u32 numOut = 0;

	// Initialize normals
	b3StackArray<b3Observation, 32> tempObservations;
	for (u32 i = 0; i < numIn; ++i)
	{
		b3WorldManifold wm;
		wm.Initialize(inManifolds + i, radiusA, xfA, radiusB, xfB);

		for (u32 j = 0; j < wm.pointCount; ++j)
		{
			b3Observation obs;
			obs.manifold = i;
			obs.manifoldPoint = j;
			obs.point = wm.points[j].normal;
			obs.cluster = B3_NULL_CLUSTER;

			tempObservations.PushBack(obs);
		}
	}

	// Initialize clusters
	b3StackArray<b3Cluster, 3> tempClusters;
	b3InitializeClusters(tempClusters, tempObservations);

	// Cluster 
	b3StackArray<b3Cluster, 3> clusters;
	b3StackArray<b3Observation, 32> observations;
	b3Clusterize(clusters, observations, tempClusters, tempObservations);

	B3_ASSERT(clusters.Count() <= 3);

	for (u32 i = 0; i < clusters.Count(); ++i)
	{		
		// Gather manifold points.
		b3Vec3 center;
		center.SetZero();
		
		b3Vec3 normal;
		normal.SetZero();

		b3StackArray<b3ClusterVertex, 32> polygonB;
		for (u32 j = 0; j < observations.Count(); ++j)
		{
			b3Observation& o = observations[j];
			if (o.cluster != i)
			{
				continue;
			}

			const b3Manifold* m = inManifolds + o.manifold;
			const b3ManifoldPoint* mp = m->points + o.manifoldPoint;

			b3WorldManifoldPoint wmp;
			wmp.Initialize(mp, radiusA, xfA, radiusB, xfB);

			b3ClusterVertex cv;
			cv.position = wmp.point;
			cv.clipIndex = j;
			polygonB.PushBack(cv);

			center += wmp.point;
			normal += o.point;
		}

		if (polygonB.IsEmpty())
		{
			continue;
		}

		center /= float32(polygonB.Count());
		normal.Normalize();

		B3_ASSERT(numOut < B3_MAX_MANIFOLDS);
		b3Manifold* manifold = outManifolds + numOut;
		++numOut;

		// Reduce.
		// Ensure deepest point is contained.
		u32 minIndex = 0;
		float32 minSeparation = B3_MAX_FLOAT;
		for (u32 j = 0; j < polygonB.Count(); ++j)
		{
			const b3Observation* o = observations.Get(polygonB[j].clipIndex);
			const b3Manifold* inManifold = inManifolds + o->manifold;
			const b3ManifoldPoint* inPoint = inManifold->points + o->manifoldPoint;

			b3WorldManifoldPoint wmp;
			wmp.Initialize(inPoint, radiusA, xfA, radiusB, xfB);

			float32 separation = wmp.separation;
			if (separation < minSeparation)
			{
				minIndex = j;
				minSeparation = separation;
			}

			polygonB[j].position = polygonB[j].position + separation * normal;
		}
		
		b3StackArray<b3ClusterVertex, 32> reducedB;
		b3ReducePolygon(reducedB, polygonB, minIndex, normal);
		for (u32 j = 0; j < reducedB.Count(); ++j)
		{
			b3ClusterVertex v = reducedB[j];
			u32 inIndex = v.clipIndex;

			const b3Observation* o = observations.Get(inIndex);
			const b3Manifold* inManifold = inManifolds + o->manifold;
			const b3ManifoldPoint* inPoint = inManifold->points + o->manifoldPoint;

			manifold->points[j] = *inPoint;
		}

		manifold->pointCount = reducedB.Count();
	}

	B3_ASSERT(numOut <= B3_MAX_MANIFOLDS);
	return numOut;
}
