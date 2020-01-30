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

#ifndef B3_SPARSE_MAT_33_VIEW_H
#define B3_SPARSE_MAT_33_VIEW_H

#include <bounce/sparse/sparse_mat33.h>
#include <bounce/sparse/sparse.h>

struct b3ArrayRowValue
{
	u32 column;
	b3Mat33 value;
};

struct b3RowValueArray
{
	u32 count;
	b3ArrayRowValue* values;
};

// A read-only sparse matrix.
struct b3SparseMat33View
{
	// Construct this sparse matrix view from a sparse matrix.
	b3SparseMat33View(const b3SparseMat33& _m);

	// Destruct this matrix view.
	~b3SparseMat33View();

	// Read an indexed element from this matrix.
	const b3Mat33& operator()(u32 i, u32 j) const;

	// Return the total number of elements in the original matrix.
	u32 GetElementCount() const
	{
		return 3 * rowCount * 3 * rowCount;
	}

	// Return an element in the original matrix given the element indices 
	// in the corresponding original matrix.
	scalar GetElement(u32 i, u32 j) const
	{
		B3_ASSERT(i < 3 * rowCount);
		B3_ASSERT(j < 3 * rowCount);

		u32 i0 = i / 3;
		u32 j0 = j / 3;

		const b3Mat33& a = (*this)(i0, j0);

		u32 ii = i - 3 * i0;
		u32 jj = j - 3 * j0;

		return a(ii, jj);
	}

	// Create the original matrix.
	// The output matrix is stored in column-major order.
	// Use the function GetElementCount() for computing the required output memory size.
	void CreateMatrix(scalar* out) const;

	u32 rowCount;
	b3RowValueArray* rows;
};

inline b3SparseMat33View::b3SparseMat33View(const b3SparseMat33& _m)
{
	rowCount = _m.rowCount;
	rows = (b3RowValueArray*)b3FrameAllocator_sparseAllocator->Allocate(rowCount * sizeof(b3RowValueArray));
	for (u32 i = 0; i < _m.rowCount; ++i)
	{
		b3RowValueList* rowList = _m.rows + i;
		b3RowValueArray* rowArray = rows + i;

		rowArray->count = rowList->count;
		rowArray->values = (b3ArrayRowValue*)b3FrameAllocator_sparseAllocator->Allocate(rowArray->count * sizeof(b3ArrayRowValue));

		u32 valueIndex = 0;
		for (b3RowValue* v = rowList->head; v; v = v->next)
		{
			rowArray->values[valueIndex].column = v->column;
			rowArray->values[valueIndex].value = v->value;
			++valueIndex;
		}
	}
}

inline b3SparseMat33View::~b3SparseMat33View()
{
	for (u32 i = 0; i < rowCount; ++i)
	{
		b3RowValueArray* rowArray = rows + i;
		b3FrameAllocator_sparseAllocator->Free(rowArray->values);
	}
	b3FrameAllocator_sparseAllocator->Free(rows);
}

inline const b3Mat33& b3SparseMat33View::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);
	
	b3RowValueArray* vs = rows + i;

	for (u32 k = 0; k < vs->count; ++k)
	{
		b3ArrayRowValue* rv = vs->values + k;
		if (rv->column == j)
		{
			return rv->value;
		}
	}

	return b3Mat33_zero;
}

inline void b3SparseMat33View::CreateMatrix(scalar* out) const
{
	u32 AM = 3 * rowCount;
	u32 AN = AM;
	scalar* A = out;

	for (u32 i = 0; i < AM * AN; ++i)
	{
		A[i] = scalar(0);
	}

	for (u32 i = 0; i < rowCount; ++i)
	{
		b3RowValueArray* vs = rows + i;

		for (u32 k = 0; k < vs->count; ++k)
		{
			b3ArrayRowValue* v = vs->values + k;

			u32 j = v->column;
			b3Mat33 a = v->value;

			for (u32 ii = 0; ii < 3; ++ii)
			{
				for (u32 jj = 0; jj < 3; ++jj)
				{
					u32 row = 3 * i + ii;
					u32 col = 3 * j + jj;

					A[row + AM * col] = a(ii, jj);
				}
			}
		}
	}
}

inline void b3Mul(b3DenseVec3& out, const b3SparseMat33View& A, const b3DenseVec3& v)
{
	B3_ASSERT(A.rowCount == out.n);

	out.SetZero();

	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3RowValueArray* rowArray = A.rows + i;

		for (u32 k = 0; k < rowArray->count; ++k)
		{
			b3ArrayRowValue* rv = rowArray->values + k;

			u32 j = rv->column;
			b3Mat33 a = rv->value;

			out[i] += a * v[j];
		}
	}
}

inline b3DenseVec3 operator*(const b3SparseMat33View& A, const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Mul(result, A, v);
	return result;
}

#endif
