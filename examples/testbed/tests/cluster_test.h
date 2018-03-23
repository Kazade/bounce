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
		for (u32 i = 0; i < 90; ++i)
		{
			float32 x = RandomFloat(-1.0f, 1.0f);
			float32 y = RandomFloat(-1.0f, 1.0f);
			float32 z = RandomFloat(-1.0f, 1.0f);
			
			b3Observation obs;
			obs.point.Set(x, y, z);
			obs.point = b3Normalize(obs.point);
			obs.cluster = B3_NULL_CLUSTER;

			m_solver.AddObservation(obs);
		}
		
		// Initialize clusters
		m_solver.Solve();

		const b3Array<b3Cluster>& clusters = m_solver.GetClusters();
		
		m_colors.Resize(clusters.Count());
		
		for (u32 i = 0; i < clusters.Count(); ++i)
		{
			m_colors[i] = b3Color(RandomFloat(0.0f, 1.0f), RandomFloat(0.0f, 1.0f), RandomFloat(0.0f, 1.0f));
		}
	}

	void Step()
	{
		const b3Array<b3Observation>& observations = m_solver.GetObservations();
		const b3Array<b3Cluster>& clusters = m_solver.GetClusters();
		
		for (u32 i = 0; i < clusters.Count(); ++i)
		{
			g_debugDraw->DrawSegment(b3Vec3(0, 0, 0), clusters[i].centroid, b3Color(1, 1, 1));
			g_debugDraw->DrawPoint(clusters[i].centroid, 4.0f, m_colors[i]);

			for (u32 j = 0; j < observations.Count(); ++j)
			{
				b3Observation obs = observations[j];
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

	b3ClusterSolver m_solver;

	b3StackArray<b3Color, 32> m_colors;
};

#endif
