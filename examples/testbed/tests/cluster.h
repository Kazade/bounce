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

class Cluster : public Test
{
public:
	enum
	{
		e_count = 64
	};

	Cluster()
	{
		Generate();
	}

	void Generate()
	{
		for (u32 i = 0; i < e_count; ++i)
		{
			float32 x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float32 z = 3.0f * RandomFloat(-1.0f, 1.0f);

			b3Vec3 p(x, y, z);
			m_points[i] = p;
		}

		for (u32 i = 0; i < B3_MAX_MANIFOLDS; ++i)
		{
			float32 r = RandomFloat(0.0f, 1.0f);
			float32 g = RandomFloat(0.0f, 1.0f);
			float32 b = RandomFloat(0.0f, 1.0f);

			b3Color c(r, g, b);
			m_colors[i] = c;
		}
	}
	
	void KeyDown(int button)
	{
		if (button == GLFW_KEY_G)
		{
			Generate();
		}
	}

	void Step()
	{
		b3ClusterSolver cluster;

		// Initialize observations
		for (u32 i = 0; i < e_count; ++i)
		{
			b3Observation obs;
			obs.point = m_points[i];
			obs.point = b3Normalize(obs.point);
			obs.cluster = B3_NULL_CLUSTER;

			cluster.AddObservation(obs);
		}

		// Run cluster
		cluster.Solve();

		// Draw
		const b3Array<b3Observation>& observations = cluster.GetObservations();
		
		const b3Array<b3Cluster>& clusters = cluster.GetClusters();
		B3_ASSERT(clusters.Count() <= B3_MAX_MANIFOLDS);

		for (u32 i = 0; i < clusters.Count(); ++i)
		{
			b3Vec3 centroid = clusters[i].centroid;

			g_draw->DrawSegment(b3Vec3_zero, centroid, b3Color_white);
			g_draw->DrawPoint(centroid, 4.0f, m_colors[i]);

			for (u32 j = 0; j < observations.Count(); ++j)
			{
				b3Observation obs = observations[j];
				if (obs.cluster == i)
				{
					g_draw->DrawPoint(obs.point, 4.0f, m_colors[i]);
				}
			}
		}
		
		g_draw->DrawString(b3Color_white, "G - Generate a random cluster");
		g_draw->DrawString(b3Color_white, "Iterations = %d", cluster.GetIterations());
	}

	static Test* Create()
	{
		return new Cluster();
	}

	b3Vec3 m_points[e_count];
	b3Color m_colors[B3_MAX_MANIFOLDS];
};

#endif