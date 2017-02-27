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

#ifndef CLUSTER_H
#define CLUSTER_H

#include <bounce/dynamics/contacts/contact_cluster.h>

extern DebugDraw* g_debugDraw;
extern Camera g_camera;

class Cluster : public Test
{
public:
	Cluster()
	{
		g_camera.m_zoom = 10.0f;

		// Initialize observations
		b3StackArray<b3Observation, 256> tempObservations;
		tempObservations.Resize(90);
		for (u32 i = 0; i < tempObservations.Count(); ++i)
		{
			float32 x = RandomFloat(-1.0f, 1.0f);
			float32 y = RandomFloat(-1.0f, 1.0f);
			float32 z = RandomFloat(-1.0f, 1.0f);
			
			tempObservations[i].point.Set(x, y, z);
			tempObservations[i].point = b3Normalize(tempObservations[i].point);
			tempObservations[i].cluster = 0xFFFFFFFF;
		}
		
		// Initialize clusters
		b3StackArray<b3Cluster, 3> tempClusters;
		b3InitializeClusters(tempClusters, tempObservations);

		// Clusterize
		b3Clusterize(m_clusters, m_observs, tempClusters, tempObservations);

		for (u32 i = 0; i < m_clusters.Count(); ++i)
		{
			m_colors[i] = b3Color(RandomFloat(0.0f, 1.0f), RandomFloat(0.0f, 1.0f), RandomFloat(0.0f, 1.0f));
		}
	}

	void Step()
	{
		for (u32 i = 0; i < m_clusters.Count(); ++i)
		{
			g_debugDraw->DrawSegment(b3Vec3(0, 0, 0), m_clusters[i].centroid, b3Color(1, 1, 1));
			g_debugDraw->DrawPoint(m_clusters[i].centroid, 4.0f, m_colors[i]);

			for (u32 j = 0; j < m_observs.Count(); ++j)
			{
				b3Observation obs = m_observs[j];
				if (obs.cluster == i)
				{
					g_debugDraw->DrawPoint(obs.point, 4.0f, m_colors[i]);
				}
			}
		}
	}

	static Test* Create()
	{
		return new Cluster();
	}

	b3StackArray<b3Observation, 256> m_observs;
	b3StackArray<b3Cluster, 3> m_clusters;
	b3Color m_colors[3];
};

#endif
