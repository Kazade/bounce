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

#include <bounce/collision/time_of_impact.h>
#include <bounce/collision/gjk/gjk.h>

u32 b3_toiCalls = 0;
u32 b3_toiMaxIters = 0;

// Compute the closest point on a segment to a point. 
static b3Vec3 b3ClosestPointOnSegment(const b3Vec3& Q,
	const b3Vec3& A, const b3Vec3& B)
{
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	if (v <= scalar(0))
	{
		return A;
	}

	if (u <= scalar(0))
	{
		return B;
	}

	scalar w = b3Dot(AB, AB);
	if (w <= B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		return A;
	}

	scalar den = scalar(1) / w;
	b3Vec3 P = den * (u * A + v * B);
	return P;
}

// Compute the closest points between two segments.
static void b3ClosestPoints(b3Vec3& C1, b3Vec3& C2,
	const b3Vec3& P1, const b3Vec3& Q1, const b3Vec3& E1, scalar L1,
	const b3Vec3& P2, const b3Vec3& Q2, const b3Vec3& E2, scalar L2)
{
	if (L1 < B3_LINEAR_SLOP && L2 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = P2;
		return;
	}

	if (L1 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = b3ClosestPointOnSegment(P1, P2, Q2);
		return;
	}

	if (L2 < B3_LINEAR_SLOP)
	{
		C1 = b3ClosestPointOnSegment(P2, P1, Q1);
		C2 = P2;
		return;
	}

	B3_ASSERT(L1 > scalar(0));
	b3Vec3 N1 = E1 / L1;

	B3_ASSERT(L2 > scalar(0));
	b3Vec3 N2 = E2 / L2;

	// Solve Ax = b
	// [1 -dot(n1, n2)][x1] = [-dot(n1, p1 - p2)] 
	// [dot(n2, n1) -1][x2] = [-dot(n2, p1 - p2)]
	scalar b = b3Dot(N1, N2);
	scalar den = scalar(1) - b * b;

	if (den != scalar(0))
	{
		scalar inv_den = scalar(1) / den;

		b3Vec3 E3 = P1 - P2;

		scalar d = b3Dot(N1, E3);
		scalar e = b3Dot(N2, E3);

		scalar s = inv_den * (b * e - d);
		scalar t = inv_den * (e - b * d);

		C1 = P1 + s * N1;
		C2 = P2 + t * N2;
	}
	else
	{
		C1 = P1;
		C2 = P2;
	}

	C1 = b3ClosestPointOnSegment(C1, P1, Q1);

	C2 = b3ClosestPointOnSegment(C1, P2, Q2);

	C1 = b3ClosestPointOnSegment(C2, P1, Q1);
}

// Separation function evaluator
struct b3SeparationFunction
{
	enum Type
	{
		e_points,
		e_edgeA,
		e_faceA,
		e_edgeB,
		e_faceB,
		e_edges
	};

	void Initialize(const b3SimplexCache& cache,
		const b3GJKProxy& proxyA, const b3Sweep& sweepA,
		const b3GJKProxy& proxyB, const b3Sweep& sweepB,
		scalar t1)
	{
		m_proxyA = &proxyA;
		m_proxyB = &proxyB;
		u32 count = cache.count;
		B3_ASSERT(count > 0 && count < 4);

		m_sweepA = sweepA;
		m_sweepB = sweepB;

		b3Transform xfA = m_sweepA.GetTransform(t1);
		b3Transform xfB = m_sweepB.GetTransform(t1);

		// Extract the closest features 
		b3GJKFeaturePair fp = b3GetFeaturePair(cache);

		u32 countA = fp.count1;
		u32* indexA = fp.index1;

		u32 countB = fp.count2;
		u32* indexB = fp.index2;

		if (countA == 1 && countB == 1)
		{
			m_type = e_points;

			b3Vec3 localA = m_proxyA->GetVertex(indexA[0]);
			b3Vec3 localB = m_proxyB->GetVertex(indexB[0]);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			m_axis = B - A;
			m_axis.Normalize();
		}
		else if (countA == 2 && countB == 1)
		{
			m_type = e_edgeA;

			b3Vec3 A = m_proxyA->GetVertex(indexA[0]);
			b3Vec3 B = m_proxyA->GetVertex(indexA[1]);
			b3Vec3 C = m_proxyB->GetVertex(indexB[0]);
			C = b3MulT(xfA, xfB * C);

			b3Vec3 AB = B - A;
			b3Vec3 AC = C - A;
			b3Vec3 AB_x_AC = b3Cross(AB, AC);
			b3Vec3 PC = b3Cross(AB_x_AC, AB);

			m_axis = b3Mul(xfA.rotation, PC);
			m_axis.Normalize();
		}
		else if (countA == 3)
		{
			m_type = e_faceA;

			b3Vec3 A = m_proxyA->GetVertex(indexA[0]);
			b3Vec3 B = m_proxyA->GetVertex(indexA[1]);
			b3Vec3 C = m_proxyA->GetVertex(indexA[2]);

			b3Vec3 D = m_proxyB->GetVertex(indexB[0]);
			D = b3MulT(xfA, xfB * D);

			b3Vec3 N = b3Cross(B - A, C - A);

			N = -N;

			if (b3Dot(D - A, N) < scalar(0))
			{
				N = -N;
			}

			m_axis = N;
			m_axis.Normalize();
			m_localPoint = A;
		}
		else if (countB == 2 && countA == 1)
		{
			m_type = e_edgeB;

			b3Vec3 A = m_proxyB->GetVertex(indexB[0]);
			b3Vec3 B = m_proxyB->GetVertex(indexB[1]);
			b3Vec3 C = m_proxyA->GetVertex(indexA[0]);
			C = b3MulT(xfB, xfA * C);

			b3Vec3 AB = B - A;
			b3Vec3 AC = C - A;
			b3Vec3 AB_x_AC = b3Cross(AB, AC);
			b3Vec3 PC = b3Cross(AB_x_AC, AB);

			m_axis = b3Mul(xfB.rotation, PC);
			m_axis.Normalize();
		}
		else if (countB == 3)
		{
			m_type = e_faceB;

			b3Vec3 A = m_proxyB->GetVertex(indexB[0]);
			b3Vec3 B = m_proxyB->GetVertex(indexB[1]);
			b3Vec3 C = m_proxyB->GetVertex(indexB[2]);
			b3Vec3 D = m_proxyA->GetVertex(indexA[0]);
			D = b3MulT(xfB, xfA * D);

			b3Vec3 N = b3Cross(B - A, C - A);

			N = -N;

			if (b3Dot(D - A, N) < scalar(0))
			{
				N = -N;
			}

			m_axis = N;
			m_axis.Normalize();
			m_localPoint = A;
		}
		else if (countA == 2 && countB == 2)
		{
			m_type = e_edges;

			b3Vec3 A = m_proxyA->GetVertex(indexA[0]);
			b3Vec3 B = m_proxyA->GetVertex(indexA[1]);

			b3Vec3 C = m_proxyB->GetVertex(indexB[0]);
			C = b3MulT(xfA, xfB * C);

			b3Vec3 D = m_proxyB->GetVertex(indexB[1]);
			D = b3MulT(xfA, xfB * D);

			b3Vec3 E1 = B - A;
			b3Vec3 E2 = D - C;

			scalar L1 = b3Length(E1);
			B3_ASSERT(L1 > B3_LINEAR_SLOP);

			scalar L2 = b3Length(E2);
			B3_ASSERT(L2 > B3_LINEAR_SLOP);

			// Check for almost parallel edges.
			const scalar kTol = scalar(0.005);

			b3Vec3 E1_x_E2 = b3Cross(E1, E2);
			scalar L = b3Length(E1_x_E2);
			if (L < kTol * L1 * L2)
			{
				// Edges are paralell
				// Use the closest points between the edges as separating axis.
				b3Vec3 C1, C2;
				b3ClosestPoints(C1, C2, A, B, E1, L1, C, D, E2, L2);

				C1 = xfA * C1;
				C2 = xfA * C2;

				m_axis = C2 - C1;
				m_axis.Normalize();

				m_edgesParalell = true;
				return;
			}

			m_edgesParalell = false;

			b3Vec3 C1 = scalar(0.5) * (A + B);
			b3Vec3 C2 = scalar(0.5) * (C + D);

			// Ensure consistent normal orientation to proxy B.
			b3Vec3 N = (scalar(1) / L) * E1_x_E2;
			if (b3Dot(N, C2 - C1) < scalar(0))
			{
				N = -N;
			}

			m_axis = N;
			m_localPoint = C1;
		}
		else
		{
			B3_ASSERT(false);
		}
	}

	scalar FindMinSeparation(u32* indexA, u32* indexB, scalar t) const
	{
		b3Transform xfA = m_sweepA.GetTransform(t);
		b3Transform xfB = m_sweepB.GetTransform(t);

		b3Quat qA = xfA.rotation;
		b3Quat qB = xfB.rotation;

		switch (m_type)
		{
		case e_points:
		{
			b3Vec3 axisA = b3MulC(qA, m_axis);
			b3Vec3 axisB = b3MulC(qB, -m_axis);

			*indexA = m_proxyA->GetSupportIndex(axisA);
			*indexB = m_proxyB->GetSupportIndex(axisB);

			b3Vec3 localA = m_proxyA->GetVertex(*indexA);
			b3Vec3 localB = m_proxyB->GetVertex(*indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, m_axis);
			return separation;
		}
		case e_edgeA:
		{
			b3Vec3 axisA = b3MulC(qA, m_axis);
			b3Vec3 axisB = b3MulC(qB, -m_axis);

			*indexA = m_proxyA->GetSupportIndex(axisA);
			*indexB = m_proxyB->GetSupportIndex(axisB);

			b3Vec3 localA = m_proxyA->GetVertex(*indexA);
			b3Vec3 localB = m_proxyB->GetVertex(*indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, m_axis);
			return separation;
		}
		case e_faceA:
		{
			b3Vec3 N = b3Mul(qA, m_axis);
			b3Vec3 A = xfA * m_localPoint;

			b3Vec3 axisB = b3MulC(qB, -N);

			*indexA = B3_MAX_U32;
			*indexB = m_proxyB->GetSupportIndex(axisB);

			b3Vec3 localB = m_proxyB->GetVertex(*indexB);
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, N);
			return separation;
		}
		case e_edgeB:
		{
			b3Vec3 axisB = b3MulC(qB, m_axis);
			b3Vec3 axisA = b3MulC(qA, -m_axis);

			*indexA = m_proxyA->GetSupportIndex(axisA);
			*indexB = m_proxyB->GetSupportIndex(axisB);

			b3Vec3 localA = m_proxyA->GetVertex(*indexA);
			b3Vec3 localB = m_proxyB->GetVertex(*indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(A - B, m_axis);
			return separation;
		}
		case e_faceB:
		{
			b3Vec3 N = b3Mul(qB, m_axis);
			b3Vec3 B = xfB * m_localPoint;

			b3Vec3 axisA = b3MulC(qA, -N);

			*indexA = m_proxyA->GetSupportIndex(axisA);
			*indexB = B3_MAX_U32;

			b3Vec3 localA = m_proxyA->GetVertex(*indexA);
			b3Vec3 A = xfA * localA;

			scalar separation = b3Dot(A - B, N);
			return separation;
		}
		case e_edges:
		{
			if (m_edgesParalell)
			{
				b3Vec3 axisA = b3MulC(qA, m_axis);
				b3Vec3 axisB = b3MulC(qB, -m_axis);

				*indexA = m_proxyA->GetSupportIndex(axisA);
				*indexB = m_proxyB->GetSupportIndex(axisB);

				b3Vec3 localA = m_proxyA->GetVertex(*indexA);
				b3Vec3 localB = m_proxyB->GetVertex(*indexB);

				b3Vec3 A = xfA * localA;
				b3Vec3 B = xfB * localB;

				scalar separation = b3Dot(B - A, m_axis);
				return separation;
			}

			b3Vec3 N = b3Mul(qA, m_axis);
			b3Vec3 A = xfA * m_localPoint;

			b3Vec3 axisB = b3MulC(qB, -N);

			*indexA = B3_MAX_U32;
			*indexB = m_proxyB->GetSupportIndex(axisB);

			b3Vec3 localB = m_proxyB->GetVertex(*indexB);
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, N);
			return separation;
		}
		default:
		{
			B3_ASSERT(false);
			*indexA = B3_MAX_U32;
			*indexB = B3_MAX_U32;
			return -B3_MAX_SCALAR;
		}
		}
	}

	scalar Evaluate(u32 indexA, u32 indexB, scalar t) const
	{
		b3Transform xfA = m_sweepA.GetTransform(t);
		b3Transform xfB = m_sweepB.GetTransform(t);

		b3Quat qA = xfA.rotation;
		b3Quat qB = xfB.rotation;

		switch (m_type)
		{
		case e_points:
		{
			b3Vec3 localA = m_proxyA->GetVertex(indexA);
			b3Vec3 localB = m_proxyB->GetVertex(indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, m_axis);
			return separation;
		}
		case e_edgeA:
		{
			b3Vec3 localA = m_proxyA->GetVertex(indexA);
			b3Vec3 localB = m_proxyB->GetVertex(indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, m_axis);
			return separation;
		}
		case e_faceA:
		{
			b3Vec3 N = b3Mul(qA, m_axis);
			b3Vec3 A = xfA * m_localPoint;

			b3Vec3 localB = m_proxyB->GetVertex(indexB);
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, N);
			return separation;
		}
		case e_edgeB:
		{
			b3Vec3 localA = m_proxyA->GetVertex(indexA);
			b3Vec3 localB = m_proxyB->GetVertex(indexB);

			b3Vec3 A = xfA * localA;
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(A - B, m_axis);
			return separation;
		}
		case e_faceB:
		{
			b3Vec3 N = b3Mul(qB, m_axis);
			b3Vec3 B = xfB * m_localPoint;

			b3Vec3 localA = m_proxyA->GetVertex(indexA);
			b3Vec3 A = xfA * localA;

			scalar separation = b3Dot(A - B, N);
			return separation;
		}
		case e_edges:
		{
			if (m_edgesParalell)
			{
				b3Vec3 localA = m_proxyA->GetVertex(indexA);
				b3Vec3 localB = m_proxyB->GetVertex(indexB);

				b3Vec3 A = xfA * localA;
				b3Vec3 B = xfB * localB;

				scalar separation = b3Dot(B - A, m_axis);
				return separation;
			}

			b3Vec3 N = b3Mul(qA, m_axis);
			b3Vec3 A = xfA * m_localPoint;

			b3Vec3 localB = m_proxyB->GetVertex(indexB);
			b3Vec3 B = xfB * localB;

			scalar separation = b3Dot(B - A, N);
			return separation;
		}
		default:
			B3_ASSERT(false);
			return scalar(0);
		}
	}

	const b3GJKProxy* m_proxyA;
	const b3GJKProxy* m_proxyB;
	b3Sweep m_sweepA, m_sweepB;
	Type m_type;
	b3Vec3 m_localPoint;
	b3Vec3 m_axis;
	bool m_edgesParalell;
};

// "Continuous Collision" (Erin)
// CCD via the local separating axis method, a CA improvement.
b3TOIOutput b3TimeOfImpact(const b3TOIInput& input)
{
	++b3_toiCalls;

	b3TOIOutput output;
	output.state = b3TOIOutput::e_unknown;
	output.t = input.tMax;
	output.iterations = 0;

	const b3GJKProxy& proxyA = input.proxyA;
	const b3GJKProxy& proxyB = input.proxyB;

	b3Sweep sweepA = input.sweepA;
	b3Sweep sweepB = input.sweepB;

	scalar tMax = input.tMax;

	scalar totalRadius = proxyA.radius + proxyB.radius;
	scalar target = b3Max(B3_LINEAR_SLOP, totalRadius - scalar(3) * B3_LINEAR_SLOP);
	scalar tolerance = scalar(0.25) * B3_LINEAR_SLOP;
	B3_ASSERT(target > tolerance);

	scalar t1 = scalar(0);
	const u32 kMaxIterations = 20;
	u32 iteration = 0;

	b3SimplexCache cache;
	cache.count = 0;

	for (;;)
	{
		b3Transform xfA = sweepA.GetTransform(t1);
		b3Transform xfB = sweepB.GetTransform(t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		b3GJKOutput gjkOutput = b3GJK(xfA, proxyA, xfB, proxyB, false, &cache);

		// If the shapes are overlapped, we give up on continuous collision.
		if (gjkOutput.distance == scalar(0))
		{
			// Failure!
			output.state = b3TOIOutput::e_overlapped;
			output.t = scalar(0);
			output.iterations = iteration;
			break;
		}

		if (gjkOutput.distance < target + tolerance)
		{
			// Victory!
			output.state = b3TOIOutput::e_touching;
			output.t = t1;
			output.iterations = iteration;
			break;
		}

		// Initialize the separating axis.
		b3SeparationFunction fcn;
		fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

		// Compute the TOI on the separating axis. 
		// We do this by successively resolving the deepest point. 
		// This loop is bounded by the number of vertices.
		bool done = false;
		scalar t2 = tMax;
		u32 pushBackIter = 0;
		for (;;)
		{
			// Compute the deepest point at t2. 
			// Store the closest point indices.
			u32 indexA, indexB;
			scalar s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output.state = b3TOIOutput::e_separated;
				output.t = tMax;
				output.iterations = iteration;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			scalar s1 = fcn.Evaluate(indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output.state = b3TOIOutput::e_failed;
				output.t = t1;
				output.iterations = iteration;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// t1 should hold the TOI. It can be 0.0.
				output.state = b3TOIOutput::e_touching;
				output.t = t1;
				output.iterations = iteration;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			u32 rootIteration = 0;
			scalar a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				scalar t;
				if (rootIteration & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = scalar(0.5) * (a1 + a2);
				}

				++rootIteration;

				scalar s = fcn.Evaluate(indexA, indexB, t);

				if (b3Abs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				if (rootIteration == 50)
				{
					break;
				}
			}

			++pushBackIter;

			if (pushBackIter == 64)
			{
				break;
			}
		}

		++iteration;

		if (done)
		{
			break;
		}

		if (iteration == kMaxIterations)
		{
			// Root finder got stuck.
			output.state = b3TOIOutput::e_failed;
			output.t = t1;
			output.iterations = iteration;
			break;
		}
	}

	b3_toiMaxIters = b3Max(b3_toiMaxIters, iteration);

	output.iterations = iteration;

	return output;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Brian Mirtich  
// "Conservative Advancement" 
b3TOIOutput b3TimeOfImpact(
	const b3Transform& xf1, const b3GJKProxy& proxy1, const b3Vec3& d1,
	const b3Transform& xf2, const b3GJKProxy& proxy2, const b3Vec3& d2, u32 maxIterations)
{
	scalar r1 = proxy1.radius;
	scalar r2 = proxy2.radius;
	scalar totalRadius = r1 + r2;

	scalar target = b3Max(B3_LINEAR_SLOP, totalRadius - scalar(3) * B3_LINEAR_SLOP);
	scalar tolerance = scalar(0.25) * B3_LINEAR_SLOP;
	B3_ASSERT(target > tolerance);

	b3SimplexCache cache;
	cache.count = 0;

	scalar t = scalar(0);
	b3Transform xf1t = xf1;
	b3Transform xf2t = xf2;

	u32 iteration = 0;
	for (;;)
	{
		b3GJKOutput query = b3GJK(xf1t, proxy1, xf2t, proxy2, false, &cache);

		scalar d = query.distance;

		// If the shapes are overlapped, we give up on continuous collision.
		if (d == scalar(0))
		{
			// Failure!
			b3TOIOutput output;
			output.state = b3TOIOutput::e_overlapped;
			output.t = scalar(0);
			output.iterations = iteration;
			return output;
		}

		if (d < target + tolerance)
		{
			// Victory!
			b3TOIOutput output;
			output.state = b3TOIOutput::e_touching;
			output.t = t;
			output.iterations = iteration;
			return output;
		}

		b3Vec3 p1 = query.point1;
		b3Vec3 p2 = query.point2;
		b3Vec3 n = (p2 - p1) / d;

		scalar denominator = b3Dot(d2 - d1, n);
		if (denominator >= scalar(0))
		{
			// Victory!
			b3TOIOutput output;
			output.state = b3TOIOutput::e_separated;
			output.t = scalar(1);
			output.iterations = iteration;
			return output;
		}

		// Advance sweep
		scalar bound = -denominator;
		B3_ASSERT(d >= target);
		scalar dt = (d - target) / bound;
		t += dt;

		if (t >= scalar(1))
		{
			// Victory!
			b3TOIOutput output;
			output.state = b3TOIOutput::e_separated;
			output.t = scalar(1);
			output.iterations = iteration;
			return output;
		}

		xf1t.translation = xf1.translation + t * d1;
		xf2t.translation = xf2.translation + t * d2;

		++iteration;

		if (iteration == maxIterations)
		{
			break;
		}
	}

	// Failure!
	b3TOIOutput output;
	output.state = b3TOIOutput::e_unknown;
	output.t = t;
	output.iterations = iteration;
	return output;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

// "Real Time Collision Detection", page 232.
b3TOIOutput b3TimeOfImpact(const b3AABB& aabb1, const b3Vec3& d1, const b3AABB& aabb2, const b3Vec3& d2)
{
	// If the shapes are overlapped, we give up on continuous collision.
	if (b3TestOverlap(aabb1, aabb2))
	{
		// Failure!
		b3TOIOutput output;
		output.state = b3TOIOutput::e_overlapped;
		output.t = scalar(0);
		output.iterations = 0;
		return output;
	}

	b3AABB A = aabb1;
	b3AABB B = aabb2;

	b3Vec3 d = d2 - d1;

	b3Vec3 t1(scalar(0), scalar(0), scalar(0));
	b3Vec3 t2(scalar(1), scalar(1), scalar(1));

	for (u32 i = 0; i < 3; ++i)
	{
		if (d[i] == scalar(0))
		{
			if (A.lowerBound[i] > B.upperBound[i] || A.upperBound[i] < B.lowerBound[i])
			{
				// Victory!
				b3TOIOutput output;
				output.state = b3TOIOutput::e_separated;
				output.t = scalar(1);
				output.iterations = 0;
				return output;
			}
		}

		if (d[i] < scalar(0))
		{
			if (B.upperBound[i] < A.lowerBound[i])
			{
				// Victory!
				b3TOIOutput output;
				output.state = b3TOIOutput::e_separated;
				output.t = scalar(1);
				output.iterations = 0;
				return output;
			}
			else
			{
				t2[i] = (A.lowerBound[i] - B.upperBound[i]) / d[i];
			}
		
			if (B.lowerBound[i] > A.upperBound[i])
			{
				t1[i] = (A.upperBound[i] - B.lowerBound[i]) / d[i];
				
				if (t1[i] >= scalar(1))
				{
					// Victory!
					b3TOIOutput output;
					output.state = b3TOIOutput::e_separated;
					output.t = scalar(1);
					output.iterations = 0;
					return output;
				}
			}
		}

		if (d[i] > scalar(0))
		{	
			if (B.lowerBound[i] > A.upperBound[i])
			{
				// Victory!
				b3TOIOutput output;
				output.state = b3TOIOutput::e_separated;
				output.t = scalar(1);
				output.iterations = 0;
				return output;
			}
			else
			{
				t2[i] = (A.upperBound[i] - B.lowerBound[i]) / d[i];
			}
		
			if (B.upperBound[i] < A.lowerBound[i])
			{
				t1[i] = (A.lowerBound[i] - B.upperBound[i]) / d[i];

				if (t1[i] >= scalar(1))
				{
					// Victory!
					b3TOIOutput output;
					output.state = b3TOIOutput::e_separated;
					output.t = scalar(1);
					output.iterations = 0;
					return output;
				}
			}
		}
	}

	scalar t1_max = b3Max(t1.x, b3Max(t1.y, t1.z));
	scalar t2_min = b3Min(t2.x, b3Min(t2.y, t2.z));

	if (t1_max > t2_min)
	{
		// Failure!
		b3TOIOutput output;
		output.state = b3TOIOutput::e_separated;
		output.t = scalar(0);
		output.iterations = 0;
		return output;
	}

	// Victory!
	b3TOIOutput output;
	output.state = b3TOIOutput::e_touching;
	output.t = t1_max;
	output.iterations = 0;
	return output;
}