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

#include <bounce\common\math\mat22.h>
#include <bounce\common\math\mat33.h>

b3Vec2 b3Mat22::Solve(const b3Vec2& b) const
{
	// Cramer's rule
	float32 a11 = x.x, a12 = y.x;
	float32	a21 = x.y, a22 = y.y;

	float32 det = a11 * a22 - a12 * a21;

	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	b3Vec2 x;
	x.x = det * (a22 * b.x - a12 * b.y);
	x.y = det * (a11 * b.y - a21 * b.x);
	return x;
}

b3Mat22 b3Inverse(const b3Mat22& A)
{
	float32 a11 = A.x.x, a12 = A.y.x;
	float32	a21 = A.x.y, a22 = A.y.y;

	float32 det = a11 * a22 - a12 * a21;
	
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	
	b3Mat22 B;
	B.x.x = det * a22;	B.y.x = -det * a12;
	B.x.y = -det * a21;	B.y.y = det * a11;
	return B;
}

b3Vec3 b3Mat33::Solve(const b3Vec3& b) const
{
	// Cramer's rule
	float32 det = b3Det(x, y, z);
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	
	b3Vec3 xn;
	xn.x = det * b3Det(b, y, z);
	xn.y = det * b3Det(x, b, z);
	xn.z = det * b3Det(x, y, b);
	return xn;
}

b3Mat33 b3Adjucate(const b3Mat33& A)
{
	b3Vec3 c1 = b3Cross(A.y, A.z);
	b3Vec3 c2 = b3Cross(A.z, A.x);
	b3Vec3 c3 = b3Cross(A.x, A.y);

	b3Mat33 B;
	B.x.x = c1.x; B.x.y = c2.x; B.x.z = c3.x;
	B.y.x = c1.y; B.y.y = c2.y; B.y.z = c3.y;
	B.z.x = c1.z; B.z.y = c2.z; B.z.z = c3.z;
	return B;
}

b3Mat33 b3Inverse(const b3Mat33& A)
{
	// Cofactor method
	float32 det = b3Det(A.x, A.y, A.z);
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	return det * b3Adjucate(A);
}

bool b3Solve(float32* b, float32* A, u32 n)
{
	// Gaussian Elimination.

	// Loop through the diagonal elements.
	for (u32 pivot = 0; pivot < n; ++pivot)
	{
		// Find the largest element in the current column.
		u32 maxRow = pivot;
		float32 maxElem = b3Abs(A[maxRow + n * pivot]);
		for (u32 i = pivot + 1; i < n; ++i)
		{
			float32 e = b3Abs(A[i + n * pivot]);
			if (e > maxElem)
			{
				maxElem = e;
				maxRow = i;
			}
		}

		// Singularity check.
		if (b3Abs(maxElem) <= B3_EPSILON)
		{
			return false;
		}

		// Swap rowns if not in the current row.
		if (maxRow != pivot)
		{
			// Swap the row.
			for (u32 j = 0; j < n; ++j)
			{
				float32 a0 = A[maxRow + n * j];
				A[maxRow + n * j] = A[pivot + n * j];
				A[pivot + n * j] = a0;
			}

			// Swap b elements.
			float32 b0 = b[maxRow];
			b[maxRow] = b[pivot];
			b[pivot] = b0;
		}

		// Divide current row by pivot.
		float32 invPivot = 1.0f / A[n * pivot + pivot];
		for (u32 j = 0; j < n; ++j)
		{
			A[pivot + n * j] *= invPivot;
		}
		b[pivot] *= invPivot;

		// Ensure pivot is 1.
		A[pivot + n * pivot] = 1.0f;

		// Zero pivot column in other rows. 
		for (u32 i = pivot + 1; i < n; ++i)
		{
			// Subtract multiple of pivot row from current row,
			// such that pivot column element becomes 0.
			float32 factor = A[i + n * pivot];

			// Subtract multiple of row.
			for (u32 j = 0; j < n; ++j)
			{
				A[i + n * j] -= factor * A[pivot + n * j];
			}

			b[i] -= factor * b[pivot];
		}
	}

	// Backwards substitution. 
	u32 p = n - 1;
	do
	{
		--p;		
		for (u32 j = p + 1; j < n; ++j)
		{
			b[p] -= A[p + n*j] * b[j];
		}
	} while (p > 0);

	return true;
}
