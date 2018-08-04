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

#include <bounce/dynamics/cloth/cloth_solver.h>
#include <bounce/dynamics/cloth/cloth.h>
#include <bounce/dynamics/cloth/particle.h>
#include <bounce/dynamics/cloth/force.h>
#include <bounce/dynamics/cloth/dense_vec3.h>
#include <bounce/dynamics/cloth/diag_mat33.h>
#include <bounce/dynamics/cloth/sparse_sym_mat33.h>
#include <bounce/dynamics/shapes/shape.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/math/mat.h>

// Here, we solve Ax = b using the Modified Preconditioned Conjugate Gradient (MPCG) algorithm.
// described in the paper:
// "Large Steps in Cloth Simulation - David Baraff, Andrew Witkin".

// Some improvements for the original MPCG algorithm are described in the paper:
// "On the modified conjugate gradient method in cloth simulation - Uri M. Ascher, Eddy Boxerman".

u32 b3_clothSolverIterations = 0;

b3ClothSolver::b3ClothSolver(const b3ClothSolverDef& def)
{
	m_allocator = def.stack;

	m_particleCapacity = def.particleCapacity;
	m_particleCount = 0;
	m_particles = (b3Particle**)m_allocator->Allocate(m_particleCapacity * sizeof(b3Particle*));

	m_forceCapacity = def.forceCapacity;
	m_forceCount = 0;
	m_forces = (b3Force**)m_allocator->Allocate(m_forceCapacity * sizeof(b3Force*));;

	m_constraintCapacity = def.particleCapacity;
	m_constraintCount = 0;
	m_constraints = (b3AccelerationConstraint*)m_allocator->Allocate(m_constraintCapacity * sizeof(b3AccelerationConstraint));

	m_bodyContactCapacity = def.bodyContactCapacity;
	m_bodyContactCount = 0;
	m_bodyContacts = (b3BodyContact**)m_allocator->Allocate(m_bodyContactCapacity * sizeof(b3BodyContact*));
	m_bodyVelocityConstraints = (b3ClothSolverContactVelocityConstraint*)m_allocator->Allocate(m_bodyContactCapacity * sizeof(b3ClothSolverContactVelocityConstraint));
	m_bodyPositionConstraints = (b3ClothSolverContactPositionConstraint*)m_allocator->Allocate(m_bodyContactCapacity * sizeof(b3ClothSolverContactPositionConstraint));

	m_particleContactCapacity = def.particleContactCapacity;
	m_particleContactCount = 0;
	m_particleContacts = (b3ParticleContact**)m_allocator->Allocate(m_particleContactCapacity * sizeof(b3ParticleContact*));
	m_particleVelocityConstraints = (b3ClothSolverParticleContactVelocityConstraint*)m_allocator->Allocate(m_particleContactCapacity * sizeof(b3ClothSolverParticleContactVelocityConstraint));
	m_particlePositionConstraints = (b3ClothSolverParticleContactPositionConstraint*)m_allocator->Allocate(m_particleContactCapacity * sizeof(b3ClothSolverParticleContactPositionConstraint));

	m_triangleContactCapacity = def.triangleContactCapacity;
	m_triangleContactCount = 0;
	m_triangleContacts = (b3TriangleContact**)m_allocator->Allocate(m_triangleContactCapacity * sizeof(b3TriangleContact*));
	m_triangleVelocityConstraints = (b3ClothSolverTriangleContactVelocityConstraint*)m_allocator->Allocate(m_triangleContactCapacity * sizeof(b3ClothSolverTriangleContactVelocityConstraint));
	m_trianglePositionConstraints = (b3ClothSolverTriangleContactPositionConstraint*)m_allocator->Allocate(m_triangleContactCapacity * sizeof(b3ClothSolverTriangleContactPositionConstraint));
}

b3ClothSolver::~b3ClothSolver()
{
	m_allocator->Free(m_trianglePositionConstraints);
	m_allocator->Free(m_triangleVelocityConstraints);
	m_allocator->Free(m_triangleContacts);

	m_allocator->Free(m_particlePositionConstraints);
	m_allocator->Free(m_particleVelocityConstraints);
	m_allocator->Free(m_particleContacts);

	m_allocator->Free(m_bodyPositionConstraints);
	m_allocator->Free(m_bodyVelocityConstraints);
	m_allocator->Free(m_bodyContacts);

	m_allocator->Free(m_constraints);
	m_allocator->Free(m_forces);
	m_allocator->Free(m_particles);
}

void b3ClothSolver::Add(b3Particle* p)
{
	p->m_solverId = m_particleCount;
	m_particles[m_particleCount++] = p;
}

void b3ClothSolver::Add(b3Force* f)
{
	m_forces[m_forceCount++] = f;
}

void b3ClothSolver::Add(b3BodyContact* c)
{
	m_bodyContacts[m_bodyContactCount++] = c;
}

void b3ClothSolver::Add(b3ParticleContact* c)
{
	m_particleContacts[m_particleContactCount++] = c;
}

void b3ClothSolver::Add(b3TriangleContact* c)
{
	m_triangleContacts[m_triangleContactCount++] = c;
}

void b3ClothSolver::ApplyForces()
{
	for (u32 i = 0; i < m_forceCount; ++i)
	{
		m_forces[i]->Apply(&m_solverData);
	}
}

void b3AccelerationConstraint::Apply(const b3ClothSolverData* data)
{
	b3DiagMat33& sS = *data->S;
	b3DenseVec3& sz = *data->z;

	sz[i1] = z;

	b3Mat33 I; I.SetIdentity();

	switch (ndof)
	{
	case 3:
	{
		sS[i1] = I;
		break;
	}
	case 2:
	{
		sS[i1] = I - b3Outer(p, p);
		break;
	}
	case 1:
	{
		sS[i1] = I - b3Outer(p, p) - b3Outer(q, q);
		break;
	}
	case 0:
	{
		sS[i1].SetZero();
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

void b3ClothSolver::ApplyConstraints()
{
	b3DiagMat33& S = *m_solverData.S;
	b3DenseVec3& z = *m_solverData.z;

	S.SetIdentity();
	z.SetZero();

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		if (p->m_type != e_dynamicParticle)
		{
			b3AccelerationConstraint* ac = m_constraints + m_constraintCount;
			++m_constraintCount;
			ac->i1 = i;
			ac->ndof = 0;
			ac->z.SetZero();
		}
	}

	for (u32 i = 0; i < m_constraintCount; ++i)
	{
		m_constraints[i].Apply(&m_solverData);
	}
}

void b3ClothSolver::Solve(float32 dt, const b3Vec3& gravity)
{
	b3DenseVec3 sx(m_particleCount);
	b3DenseVec3 sv(m_particleCount);
	b3DenseVec3 sf(m_particleCount);
	b3DenseVec3 sy(m_particleCount);
	b3DenseVec3 sx0(m_particleCount);
	b3SparseSymMat33 dfdx(m_particleCount);
	b3SparseSymMat33 dfdv(m_particleCount);
	b3DiagMat33 S(m_particleCount);
	b3DenseVec3 z(m_particleCount);

	m_solverData.x = &sx;
	m_solverData.v = &sv;
	m_solverData.f = &sf;
	m_solverData.y = &sy;
	m_solverData.dfdx = &dfdx;
	m_solverData.dfdv = &dfdv;
	m_solverData.S = &S;
	m_solverData.z = &z;
	m_solverData.dt = dt;
	m_solverData.invdt = 1.0f / dt;

	float32 h = dt;
	float32 inv_h = m_solverData.invdt;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		sx[i] = p->m_position;
		sv[i] = p->m_velocity;
		sf[i] = p->m_force;

		// Apply weight
		if (p->m_type == e_dynamicParticle)
		{
			sf[i] += p->m_mass * gravity;
		}

		sy[i] = p->m_translation;
		sx0[i] = p->m_x;
	}

	// Integrate velocities

	// Apply internal forces
	ApplyForces();

	// Apply constraints
	ApplyConstraints();

	// Solve Ax = b, where
	// A = M - h * dfdv - h * h * dfdx
	// b = h * (f0 + h * dfdx * v0 + dfdx * y) 

	// A
	b3SparseSymMat33 A(m_particleCount);
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		A(i, i) = b3Diagonal(m_particles[i]->m_mass);
	}
	A += -h * dfdv - h * h * dfdx;

	// b
	b3DenseVec3 b = h * (sf + h * (dfdx * sv) + dfdx * sy);

	// x
	b3DenseVec3 x(m_particleCount);
	u32 iterations = 0;

	Solve(x, iterations, A, b, S, z, sx0);
	b3_clothSolverIterations = iterations;

	sv = sv + x;
	sx = sx + sy;

	// Store delta velocities to improve convergence
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->m_x = x[i];
	}

	// Initialize body contact constraints
	InitializeBodyContactConstraints();

	// Initialize particle contact constraints
	InitializeParticleContactConstraints();

	// Initialize triangle contact constraints
	InitializeTriangleContactConstraints();

	// Warm start velocity constraints
	WarmStart();

	// Solve velocity constraints
	const u32 kVelocityIterations = 8;

	for (u32 i = 0; i < kVelocityIterations; ++i)
	{
		SolveBodyContactVelocityConstraints();
		SolveParticleContactVelocityConstraints();
		SolveTriangleContactVelocityConstraints();
	}

	// Store impulses to improve convergence
	StoreImpulses();

	// Integrate positions
	sx = sx + h * sv;

	// Solve position constraints
	const u32 kPositionIterations = 2;

	bool positionSolved = false;
	for (u32 i = 0; i < kPositionIterations; ++i)
	{
		bool bodyContactsSolved = SolveBodyContactPositionConstraints();
		bool particleContactsSolved = SolveParticleContactPositionConstraints();
		bool triangleContactsSolved = SolveTriangleContactPositionConstraints();
		if (bodyContactsSolved && particleContactsSolved && triangleContactsSolved)
		{
			positionSolved = true;
			break;
		}
	}

	// Synchronize bodies
	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3Body* body = m_bodyContacts[i]->s2->GetBody();

		body->SynchronizeTransform();

		body->m_worldInvI = b3RotateToFrame(body->m_invI, body->m_xf.rotation);

		body->SynchronizeShapes();
	}

	// Copy state buffers back to the particles
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		p->m_position = sx[i];
		p->m_velocity = sv[i];
	}
}

void b3ClothSolver::Solve(b3DenseVec3& x, u32& iterations,
	const b3SparseSymMat33& A, const b3DenseVec3& b, const b3DiagMat33& S, const b3DenseVec3& z, const b3DenseVec3& y) const
{
	B3_PROFILE("Solve Ax = b");

	// P = diag(A)
	b3DiagMat33 inv_P(m_particleCount);
	A.Diagonal(inv_P);

	// Invert
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Mat33& D = inv_P[i];

		// Sylvester Criterion to ensure PD-ness
		B3_ASSERT(b3Det(D.x, D.y, D.z) > 0.0f);

		B3_ASSERT(D.x.x != 0.0f);
		float32 xx = 1.0f / D.x.x;

		B3_ASSERT(D.y.y != 0.0f);
		float32 yy = 1.0f / D.y.y;

		B3_ASSERT(D.z.z != 0.0f);
		float32 zz = 1.0f / D.z.z;

		D = b3Diagonal(xx, yy, zz);
	}

	// I
	b3DiagMat33 I(m_particleCount);
	I.SetIdentity();

	// x = S * y + (I - S) * z 
	x = (S * y) + (I - S) * z;

	// b^ = S * (b - A * (I - S) * z)
	b3DenseVec3 b_hat = S * (b - A * ((I - S) * z));

	// b_delta = dot(b^, P^-1 * b_^)
	float32 b_delta = b3Dot(b_hat, inv_P * b_hat);

	// r = S * (b - A * x)
	b3DenseVec3 r = S * (b - A * x);

	// p = S * (P^-1 * r)
	b3DenseVec3 p = S * (inv_P * r);

	// delta_new = dot(r, p)
	float32 delta_new = b3Dot(r, p);

	// Set the tolerance.
	const float32 tolerance = 10.0f * B3_EPSILON;

	// Maximum number of iterations.
	// Stop at this iteration if diverged.
	const u32 max_iterations = 20;

	u32 iteration = 0;

	// Main iteration loop.
	for (;;)
	{
		// Divergence check.
		if (iteration >= max_iterations)
		{
			break;
		}

		// Convergence check.
		if (delta_new <= tolerance * tolerance * b_delta)
		{
			break;
		}

		// s = S * (A * p)
		b3DenseVec3 s = S * (A * p);

		// alpha = delta_new / dot(p, s)
		float32 alpha = delta_new / b3Dot(p, s);

		// x = x + alpha * p
		x = x + alpha * p;

		// r = r - alpha * s
		r = r - alpha * s;

		// h = inv_P * r
		b3DenseVec3 h = inv_P * r;

		// delta_old = delta_new
		float32 delta_old = delta_new;

		// delta_new = dot(r, h)
		delta_new = b3Dot(r, h);

		// beta = delta_new / delta_old
		float32 beta = delta_new / delta_old;

		// p = S * (h + beta * p)
		p = S * (h + beta * p);

		++iteration;
	}

	iterations = iteration;
}

// 
static B3_FORCE_INLINE float32 b3MixFriction(float32 u1, float32 u2)
{
	return b3Sqrt(u1 * u2);
}

void b3ClothSolver::InitializeBodyContactConstraints()
{
	b3DenseVec3& x = *m_solverData.x;
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3BodyContact* c = m_bodyContacts[i];
		b3ClothSolverContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;
		b3ClothSolverContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		vc->indexA = c->p1->m_solverId;
		vc->bodyB = c->s2->GetBody();

		vc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		vc->invMassB = vc->bodyB->GetInverseMass();

		vc->invIA.SetZero();
		vc->invIB = vc->bodyB->GetWorldInverseInertia();

		vc->friction = b3MixFriction(c->p1->m_friction, c->s2->GetFriction());

		pc->indexA = c->p1->m_solverId;
		pc->bodyB = vc->bodyB;

		pc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		pc->invMassB = vc->bodyB->m_invMass;

		pc->invIA.SetZero();
		pc->invIB = vc->bodyB->m_worldInvI;

		pc->radiusA = c->p1->m_radius;
		pc->radiusB = c->s2->m_radius;

		pc->localCenterA.SetZero();
		pc->localCenterB = pc->bodyB->m_sweep.localCenter;

		pc->localNormalA = c->localNormal1;
		pc->localPointA = c->localPoint1;
		pc->localPointB = c->localPoint2;
	}

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3BodyContact* c = m_bodyContacts[i];
		b3ClothSolverContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;
		b3ClothSolverContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = bodyB->m_sweep.worldCenter;

		b3Quat qA; qA.SetIdentity();
		b3Quat qB = bodyB->m_sweep.orientation;

		b3Vec3 localCenterA = pc->localCenterA;
		b3Vec3 localCenterB = pc->localCenterB;

		b3Transform xfA;
		xfA.rotation = b3QuatMat33(qA);
		xfA.position = xA - b3Mul(xfA.rotation, localCenterA);

		b3Transform xfB;
		xfB.rotation = b3QuatMat33(qB);
		xfB.position = xB - b3Mul(xfB.rotation, localCenterB);

		b3BodyContactWorldPoint wp;
		wp.Initialize(c, pc->radiusA, xfA, pc->radiusB, xfB);

		vc->normal = wp.normal;
		vc->tangent1 = c->t1;
		vc->tangent2 = c->t2;
		vc->point = wp.point;

		b3Vec3 point = vc->point;

		b3Vec3 rA = point - xA;
		b3Vec3 rB = point - xB;

		vc->rA = rA;
		vc->rB = rB;

		vc->normalImpulse = c->normalImpulse;
		vc->tangentImpulse = c->tangentImpulse;

		{
			b3Vec3 n = vc->normal;

			b3Vec3 rnA = b3Cross(rA, n);
			b3Vec3 rnB = b3Cross(rB, n);

			float32 K = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);

			vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;
			vc->velocityBias = 0.0f;
		}

		{
			b3Vec3 t1 = vc->tangent1;
			b3Vec3 t2 = vc->tangent2;

			b3Vec3 rn1A = b3Cross(rA, t1);
			b3Vec3 rn1B = b3Cross(rB, t1);
			b3Vec3 rn2A = b3Cross(rA, t2);
			b3Vec3 rn2B = b3Cross(rB, t2);

			// dot(t1, t2) = 0
			// J1_l1 * M1 * J2_l1 = J1_l2 * M2 * J2_l2 = 0
			float32 k11 = mA + mB + b3Dot(iA * rn1A, rn1A) + b3Dot(iB * rn1B, rn1B);
			float32 k12 = b3Dot(iA * rn1A, rn2A) + b3Dot(iB * rn1B, rn2B);
			float32 k22 = mA + mB + b3Dot(iA * rn2A, rn2A) + b3Dot(iB * rn2B, rn2B);

			b3Mat22 K;
			K.x.Set(k11, k12);
			K.y.Set(k12, k22);

			vc->tangentMass = b3Inverse(K);
		}
	}
}

void b3ClothSolver::InitializeParticleContactConstraints()
{
	b3DenseVec3& x = *m_solverData.x;
	b3DenseVec3& v = *m_solverData.v;

	float32 inv_dt = m_solverData.invdt;

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ParticleContact* c = m_particleContacts[i];
		b3ClothSolverParticleContactVelocityConstraint* vc = m_particleVelocityConstraints + i;
		b3ClothSolverParticleContactPositionConstraint* pc = m_particlePositionConstraints + i;

		vc->indexA = c->p1->m_solverId;
		vc->indexB = c->p2->m_solverId;

		vc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		vc->invMassB = c->p2->m_type == e_staticParticle ? 0.0f : c->p2->m_invMass;

		vc->friction = b3MixFriction(c->p1->m_friction, c->p2->m_friction);

		pc->indexA = c->p1->m_solverId;
		pc->indexB = c->p2->m_solverId;

		pc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		pc->invMassB = c->p2->m_type == e_staticParticle ? 0.0f : c->p2->m_invMass;

		pc->radiusA = c->p1->m_radius;
		pc->radiusB = c->p2->m_radius;
	}

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ParticleContact* c = m_particleContacts[i];
		b3ClothSolverParticleContactVelocityConstraint* vc = m_particleVelocityConstraints + i;
		b3ClothSolverParticleContactPositionConstraint* pc = m_particlePositionConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];

		b3ParticleContactWorldPoint wp;
		wp.Initialize(c);

		vc->normal = wp.normal;
		vc->tangent1 = c->t1;
		vc->tangent2 = c->t2;
		vc->point = wp.point;

		b3Vec3 point = vc->point;

		vc->normalImpulse = c->normalImpulse;
		vc->tangentImpulse = c->tangentImpulse;

		{
			b3Vec3 n = vc->normal;

			float32 K = mA + mB;

			vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;

			vc->velocityBias = 0.0f;
		}

		{
			b3Vec3 t1 = vc->tangent1;
			b3Vec3 t2 = vc->tangent2;

			float32 k11 = mA + mB;
			float32 k12 = 0.0f;
			float32 k22 = mA + mB;

			b3Mat22 K;
			K.x.Set(k11, k12);
			K.y.Set(k12, k22);

			vc->tangentMass = b3Inverse(K);
		}
	}
}

void b3ClothSolver::InitializeTriangleContactConstraints()
{
	b3DenseVec3& x = *m_solverData.x;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3TriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		vc->indexA = c->p1->m_solverId;
		vc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;

		vc->indexB = c->p2->m_solverId;
		vc->invMassB = c->p2->m_type == e_staticParticle ? 0.0f : c->p2->m_invMass;

		vc->indexC = c->p3->m_solverId;
		vc->invMassC = c->p3->m_type == e_staticParticle ? 0.0f : c->p3->m_invMass;

		vc->indexD = c->p4->m_solverId;
		vc->invMassD = c->p4->m_type == e_staticParticle ? 0.0f : c->p4->m_invMass;

		pc->indexA = c->p1->m_solverId;
		pc->invMassA = c->p1->m_type == e_staticParticle ? 0.0f : c->p1->m_invMass;
		pc->radiusA = c->p1->m_radius;

		pc->indexB = c->p2->m_solverId;
		pc->invMassB = c->p2->m_type == e_staticParticle ? 0.0f : c->p2->m_invMass;

		pc->indexC = c->p3->m_solverId;
		pc->invMassC = c->p3->m_type == e_staticParticle ? 0.0f : c->p3->m_invMass;

		pc->indexD = c->p4->m_solverId;
		pc->invMassD = c->p4->m_type == e_staticParticle ? 0.0f : c->p4->m_invMass;

		pc->triangleRadius = 0.0f;
		pc->front = c->front;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;

		u32 indexC = pc->indexC;
		float32 mC = pc->invMassC;

		u32 indexD = pc->indexD;
		float32 mD = pc->invMassD;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];
		b3Vec3 xC = x[indexC];
		b3Vec3 xD = x[indexD];

		b3Vec3 n = b3Cross(xC - xB, xD - xB);

		if (pc->front == false)
		{
			n = -n;
		}

		float32 n_len = n.Normalize();
		
		b3Mat33 I; I.SetIdentity();

		b3Mat33 N = I - b3Outer(n, n);
		if (n_len > B3_EPSILON)
		{
			N = (1.0f / n_len) * N;
		}

		b3Vec3 N_n = N * n;

		vc->JA = n;
		vc->JC = b3Cross(xD - xB, N_n);
		vc->JD = b3Cross(xC - xB, N_n);
		vc->JB = b3Cross(xC - xD, N_n) - n;

		// Compute effective mass.
		float32 K = mA + mB + mC + mD;

		vc->normalMass = K > 0.0f ? 1.0f / K : 0.0f;
		vc->normalImpulse = c->normalImpulse;
	}
}

void b3ClothSolver::WarmStart()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 P = vc->normalImpulse * vc->normal;

		vA -= mA * P;
		wA -= iA * b3Cross(vc->rA, P);

		vB += mB * P;
		wB += iB * b3Cross(vc->rB, P);

		b3Vec3 P1 = vc->tangentImpulse.x * vc->tangent1;
		b3Vec3 P2 = vc->tangentImpulse.y * vc->tangent2;

		vA -= mA * (P1 + P2);
		wA -= iA * b3Cross(vc->rA, P1 + P2);

		vB += mB * (P1 + P2);
		wB += iB * b3Cross(vc->rB, P1 + P2);

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ClothSolverParticleContactVelocityConstraint* vc = m_particleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Vec3 P = vc->normalImpulse * vc->normal;

		vA -= mA * P;
		vB += mB * P;

		b3Vec3 P1 = vc->tangentImpulse.x * vc->tangent1;
		b3Vec3 P2 = vc->tangentImpulse.y * vc->tangent2;

		vA -= mA * (P1 + P2);
		vB += mB * (P1 + P2);

		v[indexA] = vA;
		v[indexB] = vB;
	}

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;
		u32 indexC = vc->indexC;
		u32 indexD = vc->indexD;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];
		b3Vec3 vC = v[indexC];
		b3Vec3 vD = v[indexD];

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;
		float32 mC = vc->invMassC;
		float32 mD = vc->invMassD;

		b3Vec3 PA = vc->normalImpulse * vc->JA;
		b3Vec3 PB = vc->normalImpulse * vc->JB;
		b3Vec3 PC = vc->normalImpulse * vc->JC;
		b3Vec3 PD = vc->normalImpulse * vc->JD;

		vA += mA * PA;
		vB += mB * PB;
		vC += mC * PC;
		vD += mD * PD;

		v[indexA] = vA;
		v[indexB] = vB;
		v[indexC] = vC;
		v[indexD] = vD;
	}
}

void b3ClothSolver::SolveBodyContactVelocityConstraints()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		u32 indexA = vc->indexA;
		b3Body* bodyB = vc->bodyB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = bodyB->GetLinearVelocity();

		b3Vec3 wA; wA.SetZero();
		b3Vec3 wB = bodyB->GetAngularVelocity();

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Mat33 iA = vc->invIA;
		b3Mat33 iB = vc->invIB;

		b3Vec3 normal = vc->normal;
		b3Vec3 point = vc->point;

		b3Vec3 rA = vc->rA;
		b3Vec3 rB = vc->rB;

		// Solve normal constraint.
		{
			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);
			float32 Cdot = b3Dot(normal, dv);

			float32 impulse = vc->normalMass * (-Cdot + vc->velocityBias);

			float32 oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 P = impulse * normal;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		// Solve tangent constraints.
		{
			b3Vec3 dv = vB + b3Cross(wB, rB) - vA - b3Cross(wA, rA);

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

			b3Vec2 impulse = vc->tangentMass * -Cdot;
			b3Vec2 oldImpulse = vc->tangentImpulse;
			vc->tangentImpulse += impulse;

			float32 maxImpulse = vc->friction * vc->normalImpulse;
			if (b3Dot(vc->tangentImpulse, vc->tangentImpulse) > maxImpulse * maxImpulse)
			{
				vc->tangentImpulse.Normalize();
				vc->tangentImpulse *= maxImpulse;
			}

			impulse = vc->tangentImpulse - oldImpulse;

			b3Vec3 P1 = impulse.x * vc->tangent1;
			b3Vec3 P2 = impulse.y * vc->tangent2;
			b3Vec3 P = P1 + P2;

			vA -= mA * P;
			wA -= iA * b3Cross(rA, P);

			vB += mB * P;
			wB += iB * b3Cross(rB, P);
		}

		v[indexA] = vA;

		bodyB->SetLinearVelocity(vB);
		bodyB->SetAngularVelocity(wB);
	}
}

void b3ClothSolver::SolveParticleContactVelocityConstraints()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ClothSolverParticleContactVelocityConstraint* vc = m_particleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		u32 indexB = vc->indexB;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;

		b3Vec3 normal = vc->normal;
		b3Vec3 point = vc->point;

		// Solve normal constraint.
		{
			b3Vec3 dv = vB - vA;
			float32 Cdot = b3Dot(normal, dv);

			float32 impulse = vc->normalMass * (-Cdot + vc->velocityBias);

			float32 oldImpulse = vc->normalImpulse;
			vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
			impulse = vc->normalImpulse - oldImpulse;

			b3Vec3 P = impulse * normal;

			vA -= mA * P;
			vB += mB * P;
		}

		// Solve tangent constraints.
		{
			b3Vec3 dv = vB - vA;

			b3Vec2 Cdot;
			Cdot.x = b3Dot(dv, vc->tangent1);
			Cdot.y = b3Dot(dv, vc->tangent2);

			b3Vec2 impulse = vc->tangentMass * -Cdot;
			b3Vec2 oldImpulse = vc->tangentImpulse;
			vc->tangentImpulse += impulse;

			float32 maxImpulse = vc->friction * vc->normalImpulse;
			if (b3Dot(vc->tangentImpulse, vc->tangentImpulse) > maxImpulse * maxImpulse)
			{
				vc->tangentImpulse.Normalize();
				vc->tangentImpulse *= maxImpulse;
			}

			impulse = vc->tangentImpulse - oldImpulse;

			b3Vec3 P1 = impulse.x * vc->tangent1;
			b3Vec3 P2 = impulse.y * vc->tangent2;
			b3Vec3 P = P1 + P2;

			vA -= mA * P;
			vB += mB * P;
		}

		v[indexA] = vA;
		v[indexB] = vB;
	}
}

void b3ClothSolver::SolveTriangleContactVelocityConstraints()
{
	b3DenseVec3& v = *m_solverData.v;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		u32 indexA = vc->indexA;
		float32 mA = vc->invMassA;

		u32 indexB = vc->indexB;
		float32 mB = vc->invMassB;

		u32 indexC = vc->indexC;
		float32 mC = vc->invMassC;

		u32 indexD = vc->indexD;
		float32 mD = vc->invMassD;

		b3Vec3 vA = v[indexA];
		b3Vec3 vB = v[indexB];
		b3Vec3 vC = v[indexC];
		b3Vec3 vD = v[indexD];

		b3Vec3 n = vc->JA;

		// Allow some slop and prevent large corrections.
		float32 Cdot = b3Dot(n, vA - vB);

		float32 impulse = -vc->normalMass * Cdot;
		
		float32 oldImpulse = vc->normalImpulse;
		vc->normalImpulse = b3Max(vc->normalImpulse + impulse, 0.0f);
		impulse = vc->normalImpulse - oldImpulse;

		b3Vec3 PA = impulse * vc->JA;
		b3Vec3 PB = impulse * vc->JB;
		b3Vec3 PC = impulse * vc->JC;
		b3Vec3 PD = impulse * vc->JD;

		vA += mA * PA;
		vB += mB * PB;
		vC += mC * PC;
		vD += mD * PD;

		v[indexA] = vA;
		v[indexB] = vB;
		v[indexC] = vC;
		v[indexD] = vD;
	}
}

void b3ClothSolver::StoreImpulses()
{
	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3BodyContact* c = m_bodyContacts[i];
		b3ClothSolverContactVelocityConstraint* vc = m_bodyVelocityConstraints + i;

		c->normalImpulse = vc->normalImpulse;
		c->tangentImpulse = vc->tangentImpulse;
	}

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ParticleContact* c = m_particleContacts[i];
		b3ClothSolverParticleContactVelocityConstraint* vc = m_particleVelocityConstraints + i;

		c->normalImpulse = vc->normalImpulse;
		c->tangentImpulse = vc->tangentImpulse;
	}

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3TriangleContact* c = m_triangleContacts[i];
		b3ClothSolverTriangleContactVelocityConstraint* vc = m_triangleVelocityConstraints + i;

		c->normalImpulse = vc->normalImpulse;
	}
}

struct b3ClothSolverBodyContactSolverPoint
{
	void Initialize(const b3ClothSolverContactPositionConstraint* pc, const b3Transform& xfA, const b3Transform& xfB)
	{
		b3Vec3 cA = b3Mul(xfA, pc->localPointA);
		b3Vec3 cB = b3Mul(xfB, pc->localPointB);

		float32 rA = pc->radiusA;
		float32 rB = pc->radiusB;

		b3Vec3 d = cB - cA;
		float32 distance = b3Length(d);

		b3Vec3 nA(0.0f, 1.0f, 0.0f);
		if (distance > B3_EPSILON)
		{
			nA = d / distance;
		}

		b3Vec3 pA = cA + rA * nA;
		b3Vec3 pB = cB - rB * nA;

		point = 0.5f * (pA + pB);
		normal = nA;
		separation = distance - rA - rB;
	}

	b3Vec3 normal;
	b3Vec3 point;
	float32 separation;
};

bool b3ClothSolver::SolveBodyContactPositionConstraints()
{
	b3DenseVec3& x = *m_solverData.v;

	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_bodyContactCount; ++i)
	{
		b3ClothSolverContactPositionConstraint* pc = m_bodyPositionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		b3Mat33 iA = pc->invIA;
		b3Vec3 localCenterA = pc->localCenterA;

		b3Body* bodyB = pc->bodyB;
		float32 mB = pc->invMassB;
		b3Mat33 iB = pc->invIB;
		b3Vec3 localCenterB = pc->localCenterB;

		b3Vec3 cA = x[indexA];
		b3Quat qA; qA.SetIdentity();

		b3Vec3 cB = bodyB->m_sweep.worldCenter;
		b3Quat qB = bodyB->m_sweep.orientation;

		// Solve normal constraint
		b3Transform xfA;
		xfA.rotation = b3QuatMat33(qA);
		xfA.position = cA - b3Mul(xfA.rotation, localCenterA);

		b3Transform xfB;
		xfB.rotation = b3QuatMat33(qB);
		xfB.position = cB - b3Mul(xfB.rotation, localCenterB);

		b3ClothSolverBodyContactSolverPoint cpcp;
		cpcp.Initialize(pc, xfA, xfB);

		b3Vec3 normal = cpcp.normal;
		b3Vec3 point = cpcp.point;
		float32 separation = cpcp.separation;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);

		// Compute effective mass.
		b3Vec3 rA = point - cA;
		b3Vec3 rB = point - cB;

		b3Vec3 rnA = b3Cross(rA, normal);
		b3Vec3 rnB = b3Cross(rB, normal);
		float32 K = mA + mB + b3Dot(rnA, iA * rnA) + b3Dot(rnB, iB * rnB);

		// Compute normal impulse.
		float32 impulse = K > 0.0f ? -C / K : 0.0f;
		b3Vec3 P = impulse * normal;

		cA -= mA * P;
		qA -= b3Derivative(qA, iA * b3Cross(rA, P));
		qA.Normalize();

		cB += mB * P;
		qB += b3Derivative(qB, iB * b3Cross(rB, P));
		qB.Normalize();

		x[indexA] = cA;

		bodyB->m_sweep.worldCenter = cB;
		bodyB->m_sweep.orientation = qB;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}

struct b3ClothSolverParticleContactSolverPoint
{
	void Initialize(const b3Vec3& cA, float32 rA, const b3Vec3& cB, float32 rB)
	{
		b3Vec3 d = cB - cA;
		float32 distance = b3Length(d);

		b3Vec3 nA(0.0f, 1.0f, 0.0f);
		if (distance > B3_EPSILON)
		{
			nA = d / distance;
		}

		b3Vec3 pA = cA + rA * nA;
		b3Vec3 pB = cB - rB * nA;

		point = 0.5f * (pA + pB);
		normal = nA;
		separation = distance - rA - rB;
	}

	b3Vec3 point;
	b3Vec3 normal;
	float32 separation;
};

bool b3ClothSolver::SolveParticleContactPositionConstraints()
{
	b3DenseVec3& x = *m_solverData.v;

	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_particleContactCount; ++i)
	{
		b3ClothSolverParticleContactPositionConstraint* pc = m_particlePositionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		float32 rA = pc->radiusA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;
		float32 rB = pc->radiusB;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];

		b3ClothSolverParticleContactSolverPoint cpcp;
		cpcp.Initialize(xA, rA, xB, rB);

		b3Vec3 normal = cpcp.normal;
		b3Vec3 point = cpcp.point;
		float32 separation = cpcp.separation;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);

		// Compute effective mass.
		float32 K = mA + mB;

		// Compute normal impulse.
		float32 impulse = K > 0.0f ? -C / K : 0.0f;
		b3Vec3 P = impulse * normal;

		xA -= mA * P;
		xB += mB * P;

		x[indexA] = xA;
		x[indexB] = xB;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}

bool b3ClothSolver::SolveTriangleContactPositionConstraints()
{
	b3DenseVec3& x = *m_solverData.v;

	float32 minSeparation = 0.0f;

	for (u32 i = 0; i < m_triangleContactCount; ++i)
	{
		b3ClothSolverTriangleContactPositionConstraint* pc = m_trianglePositionConstraints + i;

		u32 indexA = pc->indexA;
		float32 mA = pc->invMassA;
		float32 radiusA = pc->radiusA;

		u32 indexB = pc->indexB;
		float32 mB = pc->invMassB;

		u32 indexC = pc->indexC;
		float32 mC = pc->invMassC;

		u32 indexD = pc->indexD;
		float32 mD = pc->invMassD;

		float32 triangleRadius = pc->triangleRadius;

		b3Vec3 xA = x[indexA];
		b3Vec3 xB = x[indexB];
		b3Vec3 xC = x[indexC];
		b3Vec3 xD = x[indexD];

		b3Vec3 n = b3Cross(xC - xB, xD - xB);
		if (pc->front == false)
		{
			n = -n;
		}

		float32 n_len = n.Normalize();

		float32 distance = b3Dot(n, xA - xB);
		float32 separation = distance - radiusA - triangleRadius;

		// Update max constraint error.
		minSeparation = b3Min(minSeparation, separation);

		// Allow some slop and prevent large corrections.
		float32 C = b3Clamp(B3_BAUMGARTE * (separation + B3_LINEAR_SLOP), -B3_MAX_LINEAR_CORRECTION, 0.0f);

		b3Mat33 I; I.SetIdentity();

		b3Mat33 N = I - b3Outer(n, n);
		if (n_len > B3_EPSILON)
		{
			N = (1.0f / n_len) * N;
		}

		b3Vec3 N_n = N * n;

		b3Vec3 JA = n;
		b3Vec3 JC = b3Cross(xD - xB, N_n);
		b3Vec3 JD = b3Cross(xC - xB, N_n);
		b3Vec3 JB = b3Cross(xC - xD, N_n) - n;

		// Compute effective mass.
		float32 K = mA + mB + mC + mD;

		// Compute normal impulse.
		float32 impulse = K > 0.0f ? -C / K : 0.0f;

		b3Vec3 PA = impulse * JA;
		b3Vec3 PB = impulse * JB;
		b3Vec3 PC = impulse * JC;
		b3Vec3 PD = impulse * JD;

		xA += mA * PA;
		xB += mB * PB;
		xC += mC * PC;
		xD += mD * PD;

		x[indexA] = xA;
		x[indexB] = xB;
		x[indexC] = xC;
		x[indexD] = xD;
	}

	return minSeparation >= -3.0f * B3_LINEAR_SLOP;
}