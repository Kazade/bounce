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

#ifndef B3_SPARSE_SYM_MAT_33_VIEW_H
#define B3_SPARSE_SYM_MAT_33_VIEW_H

#include <bounce/cloth/sparse_sym_mat33.h>

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

// A read-only sparse symmetric matrix.
struct b3SparseSymMat33View
{
	//
	b3SparseSymMat33View(const b3SparseSymMat33& _m);

	//
	~b3SparseSymMat33View();

	//
	const b3Mat33& operator()(u32 i, u32 j) const;
	
	//
	void Diagonal(b3DiagMat33& out) const;
		
	u32 rowCount;
	b3RowValueArray* rows;
};

inline b3SparseSymMat33View::b3SparseSymMat33View(const b3SparseSymMat33& _m)
{
	rowCount = _m.rowCount;
	rows = (b3RowValueArray*)b3Alloc(rowCount * sizeof(b3RowValueArray));
	for (u32 i = 0; i < _m.rowCount; ++i)
	{
		b3RowValueList* rowList = _m.rows + i;
		b3RowValueArray* rowArray = rows + i;

		rowArray->count = rowList->count;
		rowArray->values = (b3ArrayRowValue*)b3Alloc(rowArray->count * sizeof(b3ArrayRowValue));

		u32 valueIndex = 0;
		for (b3RowValue* v = rowList->head; v; v = v->next)
		{
			rowArray->values[valueIndex].column = v->column;
			rowArray->values[valueIndex].value = v->value;
			++valueIndex;
		}
	}
}

inline b3SparseSymMat33View::~b3SparseSymMat33View()
{
	for (u32 i = 0; i < rowCount; ++i)
	{
		b3RowValueArray* rowArray = rows + i;
		b3Free(rowArray->values);
	}
	b3Free(rows);
}

inline const b3Mat33& b3SparseSymMat33View::operator()(u32 i, u32 j) const
{
	B3_ASSERT(i < rowCount);
	B3_ASSERT(j < rowCount);

	if (i > j)
	{
		b3Swap(i, j);
	}

	b3RowValueArray* vs = rows + i;

	for (u32 c = 0; c < vs->count; ++c)
	{
		b3ArrayRowValue* rv = vs->values + c;

		u32 column = rv->column;

		if (column < j)
		{
			break;
		}

		if (column == j)
		{
			return rv->value;
		}
	}

	return b3Mat33_zero;
}

inline void b3SparseSymMat33View::Diagonal(b3DiagMat33& out) const
{
	B3_ASSERT(rowCount == out.n);

	for (u32 i = 0; i < rowCount; ++i)
	{
		out[i] = (*this)(i, i);
	}
}

inline void b3Mul(b3DenseVec3& out, const b3SparseSymMat33View& A, const b3DenseVec3& v)
{
	B3_ASSERT(A.rowCount == out.n);

	out.SetZero();

	for (u32 i = 0; i < A.rowCount; ++i)
	{
		b3RowValueArray* rowArray = A.rows + i;

		for (u32 c = 0; c < rowArray->count; ++c)
		{
			b3ArrayRowValue* rv = rowArray->values + c;

			u32 j = rv->column;
			b3Mat33 a = rv->value;

			out[i] += a * v[j];

			if (i != j)
			{
				out[j] += a * v[i];
			}
		}
	}
}

inline b3DenseVec3 operator*(const b3SparseSymMat33View& A, const b3DenseVec3& v)
{
	b3DenseVec3 result(v.n);
	b3Mul(result, A, v);
	return result;
}

#endif
