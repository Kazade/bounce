/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_mesh.h>
#include <bounce/softbody/softbody_node.h>

#include <bounce/softbody/softbody_solver.h>

#include <bounce/collision/collision.h>

#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_listeners.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/shapes/shape.h>

#include <bounce/common/draw.h>

static B3_FORCE_INLINE void b3Mul(float32* C, float32* A, u32 AM, u32 AN, float32* B, u32 BM, u32 BN)
{
	B3_ASSERT(AN == BM);

	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < BN; ++j)
		{
			C[i + AM * j] = 0.0f;

			for (u32 k = 0; k < AN; ++k)
			{
				C[i + AM * j] += A[i + AM * k] * B[k + BM * j];
			}
		}
	}
}

static B3_FORCE_INLINE void b3Transpose(float32* B, float32* A, u32 AM, u32 AN)
{
	for (u32 i = 0; i < AM; ++i)
	{
		for (u32 j = 0; j < AN; ++j)
		{
			B[j + AN * i] = A[i + AM * j];
		}
	}
}

// Compute the elasticity matrix given Young modulus and Poisson's ratio
// This is a 6 x 6 matrix 
static B3_FORCE_INLINE void b3ComputeD(float32 out[36], 
	float32 E, float32 nu)
{
	float32 lambda = (nu * E) / ((1 + nu) * (1 - 2 * nu));
	float32 mu = E / (2 * (1 + nu));

	float32 D[36] =
	{
		lambda + 2 * mu, lambda,          lambda,          0,  0,  0,
		lambda,          lambda + 2 * mu, lambda,          0,  0,  0,
		lambda,          lambda,          lambda + 2 * mu, 0,  0,  0,
		0, 				 0, 			  0, 			   mu, 0,  0,
		0, 				 0, 			  0, 			   0,  mu, 0,
		0, 			 	 0, 			  0, 			   0,  0,  mu
	};

	for (u32 i = 0; i < 36; ++i)
	{
		out[i] = D[i];
	}
}

// Compute B = S * N,
// where S is the operational matrix and N are the shape functions
// This is a 6 x 12 matrix
// A derivation and corresponding simplification for this matrix 
// can be found here:
// https://github.com/erleben/OpenTissue/blob/master/OpenTissue/dynamics/fem/fem_compute_b.h
static B3_FORCE_INLINE void b3ComputeB(float32 out[72],
	const b3Mat33& invE)
{
	// cofactor = det(E)^-1 * cofactor(E)
	b3Mat33 cofactor = b3Transpose(invE);

	// minor = det(E)^-1 * minor(E)
	b3Mat33 minor;

	minor.x.x = cofactor.x.x;
	minor.x.y = -cofactor.x.y;
	minor.x.z = cofactor.x.z;

	minor.y.x = -cofactor.y.x;
	minor.y.y = cofactor.y.y;
	minor.y.z = -cofactor.y.z;

	minor.z.x = cofactor.z.x;
	minor.z.y = -cofactor.z.y;
	minor.z.z = cofactor.z.z;

	float32 e11 = -minor.x.x; // -det(E)^-1 * det(E_11)
	float32 e12 = minor.y.x; //   det(E)^-1 * det(E_12) 
	float32 e13 = -minor.z.x; // -det(E)^-1 * det(E_13)

	float32 e21 = minor.x.y; //   det(E)^-1 * det(E_21)
	float32 e22 = -minor.y.y; // -det(E)^-1 * det(E_22)
	float32 e23 = minor.z.y; //   det(E)^-1 * det(E_23)

	float32 e31 = -minor.x.z; // -det(E)^-1 * det(E_31)
	float32 e32 = minor.y.z; //   det(E)^-1 * det(E_32) 
	float32 e33 = -minor.z.z; // -det(E)^-1 * det(E_33)

	float32 b1 = -e11 - e12 - e13;
	float32 c1 = -e21 - e22 - e23;
	float32 d1 = -e31 - e32 - e33;

	float32 b2 = e11;
	float32 c2 = e21;
	float32 d2 = e31;

	float32 b3 = e12;
	float32 c3 = e22;
	float32 d3 = e32;

	float32 b4 = e13;
	float32 c4 = e23;
	float32 d4 = e33;

	float32 B[72] =
	{
		b1, 0, 0, c1, d1, 0,
		0, c1, 0, b1, 0, d1,
		0, 0, d1, 0, b1, c1,

		b2, 0, 0, c2, d2, 0,
		0, c2, 0, b2, 0, d2,
		0, 0, d2, 0, b2, c2,

		b3, 0, 0, c3, d3, 0,
		0, c3, 0, b3, 0, d3,
		0, 0, d3, 0, b3, c3,

		b4, 0, 0, c4, d4, 0,
		0, c4, 0, b4, 0, d4,
		0, 0, d4, 0, b4, c4,
	};

	for (u32 i = 0; i < 72; ++i)
	{
		out[i] = B[i];
	}
}

static B3_FORCE_INLINE void b3SetK(b3Mat33 K[16], float32 Ke[144])
{
	b3Mat33& k11 = K[0 + 4 * 0];
	b3Mat33& k12 = K[0 + 4 * 1];
	b3Mat33& k13 = K[0 + 4 * 2];
	b3Mat33& k14 = K[0 + 4 * 3];

	b3Mat33& k21 = K[1 + 4 * 0];
	b3Mat33& k22 = K[1 + 4 * 1];
	b3Mat33& k23 = K[1 + 4 * 2];
	b3Mat33& k24 = K[1 + 4 * 3];

	b3Mat33& k31 = K[2 + 4 * 0];
	b3Mat33& k32 = K[2 + 4 * 1];
	b3Mat33& k33 = K[2 + 4 * 2];
	b3Mat33& k34 = K[2 + 4 * 3];

	b3Mat33& k41 = K[3 + 4 * 0];
	b3Mat33& k42 = K[3 + 4 * 1];
	b3Mat33& k43 = K[3 + 4 * 2];
	b3Mat33& k44 = K[3 + 4 * 3];

	// k11                  
	// a11 a12 a13 
	// a21 a22 a23 
	// a31 a32 a33
	k11.x.x = Ke[0 + 12 * 0];
	k11.x.y = Ke[1 + 12 * 0];
	k11.x.z = Ke[2 + 12 * 0];

	k11.y.x = Ke[0 + 12 * 1];
	k11.y.y = Ke[1 + 12 * 1];
	k11.y.z = Ke[2 + 12 * 1];

	k11.z.x = Ke[0 + 12 * 2];
	k11.z.y = Ke[1 + 12 * 2];
	k11.z.z = Ke[2 + 12 * 2];

	// k12
	// a14 a15 a16
	// a24 a25 a26
	// a34 a35 a36
	k12.x.x = Ke[0 + 12 * 3];
	k12.x.y = Ke[1 + 12 * 3];
	k12.x.z = Ke[2 + 12 * 3];

	k12.y.x = Ke[0 + 12 * 4];
	k12.y.y = Ke[1 + 12 * 4];
	k12.y.z = Ke[2 + 12 * 4];

	k12.z.x = Ke[0 + 12 * 5];
	k12.z.y = Ke[1 + 12 * 5];
	k12.z.z = Ke[2 + 12 * 5];

	// k13
	// a17 a18 a19
	// a27 a28 a29
	// a37 a38 a39
	k13.x.x = Ke[0 + 12 * 6];
	k13.x.y = Ke[1 + 12 * 6];
	k13.x.z = Ke[2 + 12 * 6];

	k13.y.x = Ke[0 + 12 * 7];
	k13.y.y = Ke[1 + 12 * 7];
	k13.y.z = Ke[2 + 12 * 7];

	k13.z.x = Ke[0 + 12 * 8];
	k13.z.y = Ke[1 + 12 * 8];
	k13.z.z = Ke[2 + 12 * 8];

	// k14
	// a1_10 a1_11 a1_12
	// a2_10 a2_11 a2_12
	// a3_10 a3_11 a3_12
	k14.x.x = Ke[0 + 12 * 9];
	k14.x.y = Ke[1 + 12 * 9];
	k14.x.z = Ke[2 + 12 * 9];

	k14.y.x = Ke[0 + 12 * 10];
	k14.y.y = Ke[1 + 12 * 10];
	k14.y.z = Ke[2 + 12 * 10];

	k14.z.x = Ke[0 + 12 * 11];
	k14.z.y = Ke[1 + 12 * 11];
	k14.z.z = Ke[2 + 12 * 11];

	// k21	           
	// a41 a42 a43	
	// a51 a52 a53		
	// a61 a62 a63	
	//k21.x.x = Ke[3 + 12 * 0];
	//k21.x.y = Ke[4 + 12 * 0];
	//k21.x.z = Ke[5 + 12 * 0];

	//k21.y.x = Ke[3 + 12 * 1];
	//k21.y.y = Ke[4 + 12 * 1];
	//k21.y.z = Ke[5 + 12 * 1];

	//k21.z.x = Ke[3 + 12 * 2];
	//k21.z.y = Ke[4 + 12 * 2];
	//k21.z.z = Ke[5 + 12 * 2];
	k21 = b3Transpose(k12);

	// k22	            
	// a44 a45 a46	
	// a54 a55 a56	
	// a64 a65 a66	
	k22.x.x = Ke[3 + 12 * 3];
	k22.x.y = Ke[4 + 12 * 3];
	k22.x.z = Ke[5 + 12 * 3];

	k22.y.x = Ke[3 + 12 * 4];
	k22.y.y = Ke[4 + 12 * 4];
	k22.y.z = Ke[5 + 12 * 4];

	k22.z.x = Ke[3 + 12 * 5];
	k22.z.y = Ke[4 + 12 * 5];
	k22.z.z = Ke[5 + 12 * 5];

	// k23			            
	// a47 a48 a49	
	// a57 a58 a59	
	// a67 a68 a69	
	k23.x.x = Ke[3 + 12 * 6];
	k23.x.y = Ke[4 + 12 * 6];
	k23.x.z = Ke[5 + 12 * 6];

	k23.y.x = Ke[3 + 12 * 7];
	k23.y.y = Ke[4 + 12 * 7];
	k23.y.z = Ke[5 + 12 * 7];

	k23.z.x = Ke[3 + 12 * 8];
	k23.z.y = Ke[4 + 12 * 8];
	k23.z.z = Ke[5 + 12 * 8];

	// k24            
	// a4_10 a4_11 a4_12
	// a5_10 a5_11 a5_12
	// a6_10 a6_11 a6_12
	k24.x.x = Ke[3 + 12 * 9];
	k24.x.y = Ke[4 + 12 * 9];
	k24.x.z = Ke[5 + 12 * 9];

	k24.y.x = Ke[3 + 12 * 10];
	k24.y.y = Ke[4 + 12 * 10];
	k24.y.z = Ke[5 + 12 * 10];

	k24.z.x = Ke[3 + 12 * 11];
	k24.z.y = Ke[4 + 12 * 11];
	k24.z.z = Ke[5 + 12 * 11];

	// k31	
	// a71 a72 a73	
	// a81 a82 a83	
	// a91 a92 a93	
	//k31.x.x = Ke[6 + 12 * 0];
	//k31.x.y = Ke[7 + 12 * 0];
	//k31.x.z = Ke[8 + 12 * 0];

	//k31.y.x = Ke[6 + 12 * 1];
	//k31.y.y = Ke[7 + 12 * 1];
	//k31.y.z = Ke[8 + 12 * 1];

	//k31.z.x = Ke[6 + 12 * 2];
	//k31.z.y = Ke[7 + 12 * 2];
	//k31.z.z = Ke[8 + 12 * 2];
	k31 = b3Transpose(k13);

	// k32	
	// a74 a75 a76	
	// a84 a85 a86	
	// a94 a95 a96	
	//k32.x.x = Ke[6 + 12 * 3];
	//k32.x.y = Ke[7 + 12 * 3];
	//k32.x.z = Ke[8 + 12 * 3];

	//k32.y.x = Ke[6 + 12 * 4];
	//k32.y.y = Ke[7 + 12 * 4];
	//k32.y.z = Ke[8 + 12 * 4];

	//k32.z.x = Ke[6 + 12 * 5];
	//k32.z.y = Ke[7 + 12 * 5];
	//k32.z.z = Ke[8 + 12 * 5];
	k32 = b3Transpose(k23);

	// k33 
	// a77 a78 a79 
	// a87 a88 a89 
	// a97 a98 a99 
	k33.x.x = Ke[6 + 12 * 6];
	k33.x.y = Ke[7 + 12 * 6];
	k33.x.z = Ke[8 + 12 * 6];

	k33.y.x = Ke[6 + 12 * 7];
	k33.y.y = Ke[7 + 12 * 7];
	k33.y.z = Ke[8 + 12 * 7];

	k33.z.x = Ke[6 + 12 * 8];
	k33.z.y = Ke[7 + 12 * 8];
	k33.z.z = Ke[8 + 12 * 8];

	// k34
	// a7_10 a7_11 a7_12
	// a8_10 a8_11 a8_12
	// a9_10 a9_11 a9_12
	k34.x.x = Ke[6 + 12 * 9];
	k34.x.y = Ke[7 + 12 * 9];
	k34.x.z = Ke[8 + 12 * 9];

	k34.y.x = Ke[6 + 12 * 10];
	k34.y.y = Ke[7 + 12 * 10];
	k34.y.z = Ke[8 + 12 * 10];

	k34.z.x = Ke[6 + 12 * 11];
	k34.z.y = Ke[7 + 12 * 11];
	k34.z.z = Ke[8 + 12 * 11];

	// k41	
	// a10_1 a10_2 a10_3	
	// a11_1 a11_2 a11_3	
	// a12_1 a12_2 a12_3	
	//k41.x.x = Ke[9 + 12 * 0];
	//k41.x.y = Ke[10 + 12 * 0];
	//k41.x.z = Ke[11 + 12 * 0];

	//k41.y.x = Ke[9 + 12 * 1];
	//k41.y.y = Ke[10 + 12 * 1];
	//k41.y.z = Ke[11 + 12 * 1];

	//k41.z.x = Ke[9 + 12 * 2];
	//k41.z.y = Ke[10 + 12 * 2];
	//k41.z.z = Ke[11 + 12 * 2];
	k41 = b3Transpose(k14);

	// k42 
	// a10_4 a10_5 a10_6 
	// a11_4 a11_5 a11_6 
	// a12_4 a12_5 a12_6 
	//k42.x.x = Ke[9 + 12 * 3];
	//k42.x.y = Ke[10 + 12 * 3];
	//k42.x.z = Ke[11 + 12 * 3];

	//k42.y.x = Ke[9 + 12 * 4];
	//k42.y.y = Ke[10 + 12 * 4];
	//k42.y.z = Ke[11 + 12 * 4];

	//k42.z.x = Ke[9 + 12 * 5];
	//k42.z.y = Ke[10 + 12 * 5];
	//k42.z.z = Ke[11 + 12 * 5];
	k42 = b3Transpose(k24);

	// k43 
	// a10_7 a10_8 a10_9 
	// a11_7 a11_8 a11_9 
	// a12_7 a12_8 a12_9 
	//k43.x.x = Ke[9 + 12 * 6];
	//k43.x.y = Ke[10 + 12 * 6];
	//k43.x.z = Ke[11 + 12 * 6];

	//k43.y.x = Ke[9 + 12 * 7];
	//k43.y.y = Ke[10 + 12 * 7];
	//k43.y.z = Ke[11 + 12 * 7];

	//k43.z.x = Ke[9 + 12 * 8];
	//k43.z.y = Ke[10 + 12 * 8];
	//k43.z.z = Ke[11 + 12 * 8];
	k43 = b3Transpose(k34);

	// k44
	// a10_10 a10_11 a10_12
	// a11_10 a11_11 a11_12
	// a12_10 a12_11 a12_12
	k44.x.x = Ke[9 + 12 * 9];
	k44.x.y = Ke[10 + 12 * 9];
	k44.x.z = Ke[11 + 12 * 9];

	k44.y.x = Ke[9 + 12 * 10];
	k44.y.y = Ke[10 + 12 * 10];
	k44.y.z = Ke[11 + 12 * 10];

	k44.z.x = Ke[9 + 12 * 11];
	k44.z.y = Ke[10 + 12 * 11];
	k44.z.z = Ke[11 + 12 * 11];
}

b3SoftBody::b3SoftBody(const b3SoftBodyDef& def)
{
	B3_ASSERT(def.mesh);
	B3_ASSERT(def.density > 0.0f);

	m_mesh = def.mesh;
	m_density = def.density;
	m_E = def.E;
	m_nu = def.nu;
	m_c_yield = def.c_yield;
	m_c_creep = def.c_creep;
	m_c_max = def.c_max;
	m_gravity.SetZero();
	m_world = nullptr;
	m_dt = 0.0f;

	const b3SoftBodyMesh* m = m_mesh;

	// Initialize nodes
	m_nodes = (b3SoftBodyNode*)b3Alloc(m->vertexCount * sizeof(b3SoftBodyNode));
	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		n->m_body = this;
		n->m_type = e_dynamicSoftBodyNode;
		n->m_position = m->vertices[i];
		n->m_velocity.SetZero();
		n->m_force.SetZero();
		n->m_mass = 0.0f;
		n->m_invMass = 0.0f;
		n->m_massDamping = 0.0f;
		n->m_radius = 0.0f;
		n->m_friction = 0.0f;
		n->m_userData = nullptr;
		n->m_vertex = i;
		n->m_bodyContact.active = false;

		b3AABB3 aabb;
		aabb.Set(n->m_position, 0.0f);

		n->m_broadPhaseId = m_broadPhase.CreateProxy(aabb, n);
	}

	// Compute mass
	ComputeMass();

	// Initialize elements
	m_elements = (b3SoftBodyElement*)b3Alloc(m->tetrahedronCount * sizeof(b3SoftBodyElement));
	for (u32 ei = 0; ei < m->tetrahedronCount; ++ei)
	{
		b3SoftBodyMeshTetrahedron* mt = m->tetrahedrons + ei;
		b3SoftBodyElement* e = m_elements + ei;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		b3Vec3 p1 = m->vertices[v1];
		b3Vec3 p2 = m->vertices[v2];
		b3Vec3 p3 = m->vertices[v3];
		b3Vec3 p4 = m->vertices[v4];

		float32 V = b3Volume(p1, p2, p3, p4);

		B3_ASSERT(V > 0.0f);

		b3Vec3 e1 = p2 - p1;
		b3Vec3 e2 = p3 - p1;
		b3Vec3 e3 = p4 - p1;

		b3Mat33 E(e1, e2, e3);

		e->invE = b3Inverse(E);

		// 6 x 6
		float32 D[36];
		b3ComputeD(D, m_E, m_nu);

		// 6 x 12
		float32* B = e->B;
		b3ComputeB(B, e->invE);

		// 12 x 6
		float32 BT[72];
		b3Transpose(BT, B, 6, 12);

		// 12 x 6
		float32 BT_D[72];
		b3Mul(BT_D, BT, 12, 6, D, 6, 6);

		// 12 x 12
		float32 BT_D_B[144];
		b3Mul(BT_D_B, BT_D, 12, 6, B, 6, 12);
		for (u32 i = 0; i < 144; ++i)
		{
			BT_D_B[i] *= V;
		}

		b3SetK(e->K, BT_D_B);

		// 12 x 6
		float32* P = e->P;
		b3Mul(P, BT, 12, 6, D, 6, 6);
		for (u32 i = 0; i < 72; ++i)
		{
			P[i] *= V;
		}

		for (u32 i = 0; i < 6; ++i)
		{
			e->epsilon_plastic[i] = 0.0f;
		}
	}

	// Initialize triangles
	m_triangles = (b3SoftBodyTriangle*)b3Alloc(4 * m_mesh->tetrahedronCount * sizeof(b3SoftBodyTriangle));
	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* mt = m_mesh->tetrahedrons + i;

		u32 v1 = mt->v1;
		u32 v2 = mt->v2;
		u32 v3 = mt->v3;
		u32 v4 = mt->v4;

		b3SoftBodyTriangle* t1 = m_triangles + 4 * i + 0;
		b3SoftBodyTriangle* t2 = m_triangles + 4 * i + 1;
		b3SoftBodyTriangle* t3 = m_triangles + 4 * i + 2;
		b3SoftBodyTriangle* t4 = m_triangles + 4 * i + 3;

		t1->v1 = v1;
		t1->v2 = v2;
		t1->v3 = v3;
		t1->tetrahedron = i;

		t2->v1 = v1;
		t2->v2 = v3;
		t2->v3 = v4;
		t2->tetrahedron = i;

		t3->v1 = v1;
		t3->v2 = v4;
		t3->v3 = v2;
		t3->tetrahedron = i;

		t4->v1 = v2;
		t4->v2 = v4;
		t4->v3 = v3;
		t4->tetrahedron = i;
	}
}

b3SoftBody::~b3SoftBody()
{
	b3Free(m_nodes);
	b3Free(m_elements);
	b3Free(m_triangles);
}

bool b3SoftBody::RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	u32 triangle = ~0;
	u32 tetrahedron = ~0;

	b3RayCastOutput output0;
	output0.fraction = B3_MAX_FLOAT;

	for (u32 i = 0; i < 4 * m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyTriangle* t = m_triangles + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;

		b3RayCastOutput subOutput;
		if (b3RayCast(&subOutput, &input, v1, v2, v3))
		{
			if (subOutput.fraction < output0.fraction)
			{
				triangle = i;
				tetrahedron = t->tetrahedron;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}
	}

	if (tetrahedron != ~0)
	{
		output->tetrahedron = tetrahedron;
		output->v1 = m_triangles[triangle].v1;
		output->v2 = m_triangles[triangle].v2;
		output->v3 = m_triangles[triangle].v3;
		output->fraction = output0.fraction;
		output->normal = output0.normal;

		return true;
	}

	return false;
}

b3SoftBodyNode* b3SoftBody::GetVertexNode(u32 i)
{
	B3_ASSERT(i < m_mesh->vertexCount);
	return m_nodes + i;
}

float32 b3SoftBody::GetEnergy() const
{
	float32 E = 0.0f;
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		E += n->m_mass * b3Dot(n->m_velocity, n->m_velocity);
	}
	return 0.5f * E;
}

void b3SoftBody::ComputeMass()
{
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		n->m_mass = 0.0f;
		n->m_invMass = 0.0f;
	}

	const float32 inv4 = 1.0f / 4.0f;
	const float32 rho = m_density;

	for (u32 i = 0; i < m_mesh->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* tetrahedron = m_mesh->tetrahedrons + i;

		b3Vec3 v1 = m_mesh->vertices[tetrahedron->v1];
		b3Vec3 v2 = m_mesh->vertices[tetrahedron->v2];
		b3Vec3 v3 = m_mesh->vertices[tetrahedron->v3];
		b3Vec3 v4 = m_mesh->vertices[tetrahedron->v4];

		float32 volume = b3Volume(v1, v2, v3, v4);
		B3_ASSERT(volume > 0.0f);

		float32 mass = rho * volume;

		b3SoftBodyNode* n1 = m_nodes + tetrahedron->v1;
		b3SoftBodyNode* n2 = m_nodes + tetrahedron->v2;
		b3SoftBodyNode* n3 = m_nodes + tetrahedron->v3;
		b3SoftBodyNode* n4 = m_nodes + tetrahedron->v4;

		n1->m_mass += inv4 * mass;
		n2->m_mass += inv4 * mass;
		n3->m_mass += inv4 * mass;
		n4->m_mass += inv4 * mass;
	}

	// Invert
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;
		B3_ASSERT(n->m_mass > 0.0f);
		n->m_invMass = 1.0f / n->m_mass;
	}
}

class b3SoftBodyUpdateContactsQueryListener : public b3QueryListener
{
public:
	bool ReportShape(b3Shape* shape)
	{
		b3Body* body = shape->GetBody();

		if (body->GetType() != e_staticBody)
		{
			// return true;
		}

		b3Transform xf = body->GetTransform();

		b3TestSphereOutput output;
		if (shape->TestSphere(&output, sphere, xf))
		{
			if (output.separation < bestSeparation)
			{
				bestShape = shape;
				bestSeparation = output.separation;
				bestPoint = output.point;
				bestNormal = output.normal;
			}
		}

		return true;
	}

	b3Sphere sphere;
	b3Shape* bestShape;
	float32 bestSeparation;
	b3Vec3 bestPoint;
	b3Vec3 bestNormal;
};

void b3SoftBody::UpdateContacts()
{
	B3_PROFILE("Soft Body Update Contacts");

	// Is there a world attached to this soft body?
	if (m_world == nullptr)
	{
		return;
	}

	// Create contacts 
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		if (n->m_type != e_dynamicSoftBodyNode)
		{
			continue;
		}

		b3AABB3 aabb = m_broadPhase.GetAABB(n->m_broadPhaseId);

		b3SoftBodyUpdateContactsQueryListener listener;
		listener.sphere.vertex = n->m_position;
		listener.sphere.radius = n->m_radius;
		listener.bestShape = nullptr;
		listener.bestSeparation = 0.0f;

		m_world->QueryAABB(&listener, aabb);

		if (listener.bestShape == nullptr)
		{
			n->m_bodyContact.active = false;
			continue;
		}

		b3Shape* shape = listener.bestShape;
		b3Body* body = shape->GetBody();
		float32 separation = listener.bestSeparation;
		b3Vec3 point = listener.bestPoint;
		b3Vec3 normal = -listener.bestNormal;

		b3NodeBodyContact* c = &n->m_bodyContact;

		b3NodeBodyContact c0 = *c;

		c->active = true;
		c->n1 = n;
		c->s2 = shape;
		c->normal1 = normal;
		c->localPoint1.SetZero();
		c->localPoint2 = body->GetLocalPoint(point);
		c->t1 = b3Perp(normal);
		c->t2 = b3Cross(c->t1, normal);
		c->normalImpulse = 0.0f;
		c->tangentImpulse.SetZero();

		if (c0.active == true)
		{
			c->normalImpulse = c0.normalImpulse;
			c->tangentImpulse = c0.tangentImpulse;
		}
	}
}

void b3SoftBody::Solve(float32 dt, const b3Vec3& gravity, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Soft Body Solve");

	b3SoftBodySolverDef def;
	def.body = this;

	b3SoftBodySolver solver(def);

	solver.Solve(dt, gravity, velocityIterations, positionIterations);
}

void b3SoftBody::Step(float32 dt, u32 velocityIterations, u32 positionIterations)
{
	B3_PROFILE("Soft Body Step");

	m_dt = dt;

	// Update contacts
	UpdateContacts();

	// Integrate state, solve constraints. 
	if (dt > 0.0f)
	{
		Solve(dt, m_gravity, velocityIterations, positionIterations);
	}

	// Clear forces
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		m_nodes[i].m_force.SetZero();
	}

	// Synchronize nodes
	for (u32 i = 0; i < m_mesh->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		if (n->m_type == e_staticSoftBodyNode)
		{
			continue;
		}

		n->Synchronize();
	}
}

void b3SoftBody::Draw() const
{
	const b3SoftBodyMesh* m = m_mesh;

	for (u32 i = 0; i < m->vertexCount; ++i)
	{
		b3SoftBodyNode* n = m_nodes + i;

		b3Vec3 v = n->m_position;

		if (n->m_type == e_staticSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, 4.0f, b3Color_white);
		}

		if (n->m_type == e_kinematicSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, 4.0f, b3Color_blue);
		}

		if (n->m_type == e_dynamicSoftBodyNode)
		{
			b3Draw_draw->DrawPoint(v, 4.0f, b3Color_green);
		}
	}

	for (u32 i = 0; i < m->tetrahedronCount; ++i)
	{
		b3SoftBodyMeshTetrahedron* t = m->tetrahedrons + i;

		b3Vec3 v1 = m_nodes[t->v1].m_position;
		b3Vec3 v2 = m_nodes[t->v2].m_position;
		b3Vec3 v3 = m_nodes[t->v3].m_position;
		b3Vec3 v4 = m_nodes[t->v4].m_position;

		b3Vec3 c = (v1 + v2 + v3 + v4) / 4.0f;

		float32 s = 0.9f;

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;
		v4 = s * (v4 - c) + c;

		// v1, v2, v3
		b3Draw_draw->DrawTriangle(v1, v2, v3, b3Color_black);

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n1, v1, v2, v3, b3Color_blue);

		// v1, v3, v4
		b3Draw_draw->DrawTriangle(v1, v3, v4, b3Color_black);

		b3Vec3 n2 = b3Cross(v3 - v1, v4 - v1);
		n2.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n2, v1, v3, v4, b3Color_blue);

		// v1, v4, v2
		b3Draw_draw->DrawTriangle(v1, v4, v2, b3Color_black);

		b3Vec3 n3 = b3Cross(v4 - v1, v2 - v1);
		n3.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n3, v1, v4, v2, b3Color_blue);

		// v2, v4, v3
		b3Draw_draw->DrawTriangle(v2, v4, v3, b3Color_black);

		b3Vec3 n4 = b3Cross(v4 - v2, v3 - v2);
		n4.Normalize();
		b3Draw_draw->DrawSolidTriangle(-n4, v2, v4, v3, b3Color_blue);
	}
}