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

#include <bounce/collision/shapes/sdf.h>

#include <cstdio>
#include <climits>

struct b3ByteStream
{
	b3ByteStream(const char* _stream, u32 _size)
	{
		stream = _stream;
		size = _size;
		offset = 0;
	}

	template <typename T>
	void Read(T& out)
	{
		u32 out_size = sizeof(T);
		B3_ASSERT(offset + out_size <= size);
		memcpy((char*)& out, stream + offset, out_size);
		offset += out_size;
	}

	const char* stream;
	u32 size;
	u32 offset;
};

b3SDF::b3SDF() 
{ 
}

b3SDF::~b3SDF() 
{
	for (u32 i = 0; i < m_nodeCount; ++i)
	{
		b3Free(m_nodes[i].values);
	}
	b3Free(m_nodes);

	for (u32 i = 0; i < m_cellCount; ++i)
	{
		b3Free(m_cells[i].values);
	}
	b3Free(m_cells);

	for (u32 i = 0; i < m_cellMapCount; ++i)
	{
		b3Free(m_cell_map[i].values);
	}
	b3Free(m_cell_map);
}

bool b3SDF::Load(const char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file == nullptr)
	{
		return false;
	}

	fseek(file, 0, SEEK_END);
	long file_size = ftell(file);
	fseek(file, 0, SEEK_SET);
	char* file_buffer = (char*)b3Alloc(file_size);
	fread(file_buffer, file_size, 1, file);
	fclose(file);

	b3ByteStream stream(file_buffer, file_size);

	{
		double out[6];
		stream.Read(out);
		
		m_domain.lowerBound[0] = out[0];
		m_domain.lowerBound[1] = out[1];
		m_domain.lowerBound[2] = out[2];
		
		m_domain.upperBound[0] = out[3];
		m_domain.upperBound[1] = out[4];
		m_domain.upperBound[2] = out[5];
	}

	{
		unsigned int out[3];
		stream.Read(out);
		
		m_resolution[0] = out[0];
		m_resolution[1] = out[1];
		m_resolution[2] = out[2];
	}

	{
		double out[3];
		stream.Read(out);
		
		m_cell_size[0] = out[0];
		m_cell_size[1] = out[1];
		m_cell_size[2] = out[2];
	}

	{
		double out[3];
		stream.Read(out);
		
		m_inv_cell_size[0] = out[0];
		m_inv_cell_size[1] = out[1];
		m_inv_cell_size[2] = out[2];
	}

	{
		std::size_t out;
		stream.Read(out);
		
		m_n_cells = out;
	}

	{
		std::size_t out;
		stream.Read(out);
		
		m_n_fields = out;
	}

	{
		std::size_t node_count;
		stream.Read(node_count);
		m_nodeCount = node_count;
		m_nodes = (b3SDFNodeArray*)b3Alloc(node_count * sizeof(b3SDFNodeArray));
		for (std::size_t i = 0; i < node_count; ++i)
		{
			std::size_t value_count;
			stream.Read(value_count);
			
			b3SDFNodeArray* node = m_nodes + i;
			node->count = value_count;
			node->values = (double*)b3Alloc(value_count * sizeof(double));
			for (std::size_t j = 0; j < value_count; ++j)
			{
				double& out = node->values[j];
				stream.Read(out);
			}
		}
	}

	{
		std::size_t cell_count;
		stream.Read(cell_count);
		m_cellCount = cell_count;
		m_cells = (b3SDFCellArray*)b3Alloc(cell_count * sizeof(b3SDFCellArray));
		for (std::size_t i = 0; i < cell_count; ++i)
		{
			std::size_t value_count;
			stream.Read(value_count);
			
			b3SDFCellArray* cell = m_cells + i;
			cell->count = value_count;
			cell->values = (b3Cell32*)b3Alloc(value_count * sizeof(b3Cell32));
			for (std::size_t j = 0; j < value_count; ++j)
			{
				b3Cell32& out = cell->values[j];
				stream.Read(out);
			}
		}
	}

	{
		std::size_t cell_map_count;
		stream.Read(cell_map_count);
		m_cellMapCount = cell_map_count;
		m_cell_map = (b3SDFCellMapArray*)b3Alloc(cell_map_count * sizeof(b3SDFCellMapArray));
		for (std::size_t i = 0; i < cell_map_count; ++i)
		{
			std::size_t value_count;
			stream.Read(value_count);
			
			b3SDFCellMapArray* maps = m_cell_map + i;
			maps->count = value_count;
			maps->values = (unsigned int*)b3Alloc(value_count * sizeof(unsigned int));
			for (std::size_t j = 0; j < value_count; ++j)
			{
				unsigned int& out = maps->values[j];
				stream.Read(out);
			}
		}
	}

	B3_ASSERT(stream.offset == stream.size);

	b3Free(file_buffer);

	return true;
}

b3MultiIndex b3SDF::singleToMultiIndex(unsigned int l) const
{
	unsigned int n01 = m_resolution[0] * m_resolution[1];
	unsigned int k = l / n01;
	unsigned int temp = l % n01;
	unsigned int j = temp / m_resolution[0];
	unsigned int i = temp % m_resolution[0];

	b3MultiIndex mi;
	mi.v[0] = i;
	mi.v[1] = j;
	mi.v[2] = k;

	return mi;
}

unsigned int b3SDF::multiToSingleIndex(b3MultiIndex const& ijk) const
{
	return m_resolution[1] * m_resolution[0] * ijk[2] + m_resolution[0] * ijk[1] + ijk[0];
}

b3AABB b3SDF::subdomain(b3MultiIndex const& ijk) const
{
	b3Vec3 tmp;
	tmp[0] = m_cell_size[0] * (double)ijk[0];
	tmp[1] = m_cell_size[1] * (double)ijk[1];
	tmp[2] = m_cell_size[2] * (double)ijk[2];

	b3Vec3 origin = m_domain.lowerBound + tmp;

	b3AABB aabb;
	aabb.lowerBound = origin;
	aabb.upperBound = origin + m_cell_size;

	return aabb;
}

b3AABB b3SDF::subdomain(unsigned int l) const
{
	return subdomain(singleToMultiIndex(l));
}

struct b3ShapeMatrix
{
	double& operator[](int i)
	{
		return v[i];
	}

	const double& operator[](int i) const
	{
		return v[i];
	}

	double v[32];
};

struct b3ShapeGradients
{
	void bottomRowsMul(int row, double s)
	{
		for (int i = 32 - row; i < 32; ++i)
		{
			v[i] *= s;
		}
	}

	void topRowsDiv(int row, double s)
	{
		for (int i = 0; i < row; ++i)
		{
			v[i] /= s;
		}
	}

	scalar& operator()(int i, int j)
	{
		return v[i][j];
	}

	b3Vec3 v[32];
};

static b3ShapeMatrix shape_function_(b3Vec3 const& xi, b3ShapeGradients* gradient = nullptr)
{
	b3ShapeMatrix res;

	double x = (double)xi[0];
	double y = (double)xi[1];
	double z = (double)xi[2];

	double x2 = x * x;
	double y2 = y * y;
	double z2 = z * z;

	double _1mx = 1.0 - x;
	double _1my = 1.0 - y;
	double _1mz = 1.0 - z;

	double _1px = 1.0 + x;
	double _1py = 1.0 + y;
	double _1pz = 1.0 + z;

	double _1m3x = 1.0 - 3.0 * x;
	double _1m3y = 1.0 - 3.0 * y;
	double _1m3z = 1.0 - 3.0 * z;

	double _1p3x = 1.0 + 3.0 * x;
	double _1p3y = 1.0 + 3.0 * y;
	double _1p3z = 1.0 + 3.0 * z;

	double _1mxt1my = _1mx * _1my;
	double _1mxt1py = _1mx * _1py;
	double _1pxt1my = _1px * _1my;
	double _1pxt1py = _1px * _1py;

	double _1mxt1mz = _1mx * _1mz;
	double _1mxt1pz = _1mx * _1pz;
	double _1pxt1mz = _1px * _1mz;
	double _1pxt1pz = _1px * _1pz;

	double _1myt1mz = _1my * _1mz;
	double _1myt1pz = _1my * _1pz;
	double _1pyt1mz = _1py * _1mz;
	double _1pyt1pz = _1py * _1pz;

	double _1mx2 = 1.0 - x2;
	double _1my2 = 1.0 - y2;
	double _1mz2 = 1.0 - z2;

	// Corner nodes.
	double fac = 1.0 / 64.0 * (9.0 * (x2 + y2 + z2) - 19.0);
	res[0] = fac * _1mxt1my * _1mz;
	res[1] = fac * _1pxt1my * _1mz;
	res[2] = fac * _1mxt1py * _1mz;
	res[3] = fac * _1pxt1py * _1mz;
	res[4] = fac * _1mxt1my * _1pz;
	res[5] = fac * _1pxt1my * _1pz;
	res[6] = fac * _1mxt1py * _1pz;
	res[7] = fac * _1pxt1py * _1pz;

	// Edge nodes.

	fac = 9.0 / 64.0 * _1mx2;
	double fact1m3x = fac * _1m3x;
	double fact1p3x = fac * _1p3x;
	res[8] = fact1m3x * _1myt1mz;
	res[9] = fact1p3x * _1myt1mz;
	res[10] = fact1m3x * _1myt1pz;
	res[11] = fact1p3x * _1myt1pz;
	res[12] = fact1m3x * _1pyt1mz;
	res[13] = fact1p3x * _1pyt1mz;
	res[14] = fact1m3x * _1pyt1pz;
	res[15] = fact1p3x * _1pyt1pz;

	fac = 9.0 / 64.0 * _1my2;
	double fact1m3y = fac * _1m3y;
	double fact1p3y = fac * _1p3y;
	res[16] = fact1m3y * _1mxt1mz;
	res[17] = fact1p3y * _1mxt1mz;
	res[18] = fact1m3y * _1pxt1mz;
	res[19] = fact1p3y * _1pxt1mz;
	res[20] = fact1m3y * _1mxt1pz;
	res[21] = fact1p3y * _1mxt1pz;
	res[22] = fact1m3y * _1pxt1pz;
	res[23] = fact1p3y * _1pxt1pz;

	fac = 9.0 / 64.0 * _1mz2;
	double fact1m3z = fac * _1m3z;
	double fact1p3z = fac * _1p3z;
	res[24] = fact1m3z * _1mxt1my;
	res[25] = fact1p3z * _1mxt1my;
	res[26] = fact1m3z * _1mxt1py;
	res[27] = fact1p3z * _1mxt1py;
	res[28] = fact1m3z * _1pxt1my;
	res[29] = fact1p3z * _1pxt1my;
	res[30] = fact1m3z * _1pxt1py;
	res[31] = fact1p3z * _1pxt1py;

	if (gradient)
	{
		b3ShapeGradients& dN = *gradient;

		double _9t3x2py2pz2m19 = 9.0 * (3.0 * x2 + y2 + z2) - 19.0;
		double _9tx2p3y2pz2m19 = 9.0 * (x2 + 3.0 * y2 + z2) - 19.0;
		double _9tx2py2p3z2m19 = 9.0 * (x2 + y2 + 3.0 * z2) - 19.0;
		double _18x = 18.0 * x;
		double _18y = 18.0 * y;
		double _18z = 18.0 * z;

		double _3m9x2 = 3.0 - 9.0 * x2;
		double _3m9y2 = 3.0 - 9.0 * y2;
		double _3m9z2 = 3.0 - 9.0 * z2;

		double _2x = 2.0 * x;
		double _2y = 2.0 * y;
		double _2z = 2.0 * z;

		double _18xm9t3x2py2pz2m19 = _18x - _9t3x2py2pz2m19;
		double _18xp9t3x2py2pz2m19 = _18x + _9t3x2py2pz2m19;
		double _18ym9tx2p3y2pz2m19 = _18y - _9tx2p3y2pz2m19;
		double _18yp9tx2p3y2pz2m19 = _18y + _9tx2p3y2pz2m19;
		double _18zm9tx2py2p3z2m19 = _18z - _9tx2py2p3z2m19;
		double _18zp9tx2py2p3z2m19 = _18z + _9tx2py2p3z2m19;

		dN(0, 0) = _18xm9t3x2py2pz2m19 * _1myt1mz;
		dN(0, 1) = _1mxt1mz * _18ym9tx2p3y2pz2m19;
		dN(0, 2) = _1mxt1my * _18zm9tx2py2p3z2m19;
		dN(1, 0) = _18xp9t3x2py2pz2m19 * _1myt1mz;
		dN(1, 1) = _1pxt1mz * _18ym9tx2p3y2pz2m19;
		dN(1, 2) = _1pxt1my * _18zm9tx2py2p3z2m19;
		dN(2, 0) = _18xm9t3x2py2pz2m19 * _1pyt1mz;
		dN(2, 1) = _1mxt1mz * _18yp9tx2p3y2pz2m19;
		dN(2, 2) = _1mxt1py * _18zm9tx2py2p3z2m19;
		dN(3, 0) = _18xp9t3x2py2pz2m19 * _1pyt1mz;
		dN(3, 1) = _1pxt1mz * _18yp9tx2p3y2pz2m19;
		dN(3, 2) = _1pxt1py * _18zm9tx2py2p3z2m19;
		dN(4, 0) = _18xm9t3x2py2pz2m19 * _1myt1pz;
		dN(4, 1) = _1mxt1pz * _18ym9tx2p3y2pz2m19;
		dN(4, 2) = _1mxt1my * _18zp9tx2py2p3z2m19;
		dN(5, 0) = _18xp9t3x2py2pz2m19 * _1myt1pz;
		dN(5, 1) = _1pxt1pz * _18ym9tx2p3y2pz2m19;
		dN(5, 2) = _1pxt1my * _18zp9tx2py2p3z2m19;
		dN(6, 0) = _18xm9t3x2py2pz2m19 * _1pyt1pz;
		dN(6, 1) = _1mxt1pz * _18yp9tx2p3y2pz2m19;
		dN(6, 2) = _1mxt1py * _18zp9tx2py2p3z2m19;
		dN(7, 0) = _18xp9t3x2py2pz2m19 * _1pyt1pz;
		dN(7, 1) = _1pxt1pz * _18yp9tx2p3y2pz2m19;
		dN(7, 2) = _1pxt1py * _18zp9tx2py2p3z2m19;

		dN.topRowsDiv(8, 64.0);

		double _m3m9x2m2x = -_3m9x2 - _2x;
		double _p3m9x2m2x = _3m9x2 - _2x;
		double _1mx2t1m3x = _1mx2 * _1m3x;
		double _1mx2t1p3x = _1mx2 * _1p3x;
		dN(8, 0) = _m3m9x2m2x * _1myt1mz,
			dN(8, 1) = -_1mx2t1m3x * _1mz,
			dN(8, 2) = -_1mx2t1m3x * _1my;
		dN(9, 0) = _p3m9x2m2x * _1myt1mz,
			dN(9, 1) = -_1mx2t1p3x * _1mz,
			dN(9, 2) = -_1mx2t1p3x * _1my;
		dN(10, 0) = _m3m9x2m2x * _1myt1pz,
			dN(10, 1) = -_1mx2t1m3x * _1pz,
			dN(10, 2) = _1mx2t1m3x * _1my;
		dN(11, 0) = _p3m9x2m2x * _1myt1pz,
			dN(11, 1) = -_1mx2t1p3x * _1pz,
			dN(11, 2) = _1mx2t1p3x * _1my;
		dN(12, 0) = _m3m9x2m2x * _1pyt1mz,
			dN(12, 1) = _1mx2t1m3x * _1mz,
			dN(12, 2) = -_1mx2t1m3x * _1py;
		dN(13, 0) = _p3m9x2m2x * _1pyt1mz,
			dN(13, 1) = _1mx2t1p3x * _1mz,
			dN(13, 2) = -_1mx2t1p3x * _1py;
		dN(14, 0) = _m3m9x2m2x * _1pyt1pz,
			dN(14, 1) = _1mx2t1m3x * _1pz,
			dN(14, 2) = _1mx2t1m3x * _1py;
		dN(15, 0) = _p3m9x2m2x * _1pyt1pz,
			dN(15, 1) = _1mx2t1p3x * _1pz,
			dN(15, 2) = _1mx2t1p3x * _1py;

		double _m3m9y2m2y = -_3m9y2 - _2y;
		double _p3m9y2m2y = _3m9y2 - _2y;
		double _1my2t1m3y = _1my2 * _1m3y;
		double _1my2t1p3y = _1my2 * _1p3y;
		dN(16, 0) = -_1my2t1m3y * _1mz,
			dN(16, 1) = _m3m9y2m2y * _1mxt1mz,
			dN(16, 2) = -_1my2t1m3y * _1mx;
		dN(17, 0) = -_1my2t1p3y * _1mz,
			dN(17, 1) = _p3m9y2m2y * _1mxt1mz,
			dN(17, 2) = -_1my2t1p3y * _1mx;
		dN(18, 0) = _1my2t1m3y * _1mz,
			dN(18, 1) = _m3m9y2m2y * _1pxt1mz,
			dN(18, 2) = -_1my2t1m3y * _1px;
		dN(19, 0) = _1my2t1p3y * _1mz,
			dN(19, 1) = _p3m9y2m2y * _1pxt1mz,
			dN(19, 2) = -_1my2t1p3y * _1px;
		dN(20, 0) = -_1my2t1m3y * _1pz,
			dN(20, 1) = _m3m9y2m2y * _1mxt1pz,
			dN(20, 2) = _1my2t1m3y * _1mx;
		dN(21, 0) = -_1my2t1p3y * _1pz,
			dN(21, 1) = _p3m9y2m2y * _1mxt1pz,
			dN(21, 2) = _1my2t1p3y * _1mx;
		dN(22, 0) = _1my2t1m3y * _1pz,
			dN(22, 1) = _m3m9y2m2y * _1pxt1pz,
			dN(22, 2) = _1my2t1m3y * _1px;
		dN(23, 0) = _1my2t1p3y * _1pz,
			dN(23, 1) = _p3m9y2m2y * _1pxt1pz,
			dN(23, 2) = _1my2t1p3y * _1px;

		double _m3m9z2m2z = -_3m9z2 - _2z;
		double _p3m9z2m2z = _3m9z2 - _2z;
		double _1mz2t1m3z = _1mz2 * _1m3z;
		double _1mz2t1p3z = _1mz2 * _1p3z;
		dN(24, 0) = -_1mz2t1m3z * _1my,
			dN(24, 1) = -_1mz2t1m3z * _1mx,
			dN(24, 2) = _m3m9z2m2z * _1mxt1my;
		dN(25, 0) = -_1mz2t1p3z * _1my,
			dN(25, 1) = -_1mz2t1p3z * _1mx,
			dN(25, 2) = _p3m9z2m2z * _1mxt1my;
		dN(26, 0) = -_1mz2t1m3z * _1py,
			dN(26, 1) = _1mz2t1m3z * _1mx,
			dN(26, 2) = _m3m9z2m2z * _1mxt1py;
		dN(27, 0) = -_1mz2t1p3z * _1py,
			dN(27, 1) = _1mz2t1p3z * _1mx,
			dN(27, 2) = _p3m9z2m2z * _1mxt1py;
		dN(28, 0) = _1mz2t1m3z * _1my,
			dN(28, 1) = -_1mz2t1m3z * _1px,
			dN(28, 2) = _m3m9z2m2z * _1pxt1my;
		dN(29, 0) = _1mz2t1p3z * _1my,
			dN(29, 1) = -_1mz2t1p3z * _1px,
			dN(29, 2) = _p3m9z2m2z * _1pxt1my;
		dN(30, 0) = _1mz2t1m3z * _1py,
			dN(30, 1) = _1mz2t1m3z * _1px,
			dN(30, 2) = _m3m9z2m2z * _1pxt1py;
		dN(31, 0) = _1mz2t1p3z * _1py,
			dN(31, 1) = _1mz2t1p3z * _1px,
			dN(31, 2) = _p3m9z2m2z * _1pxt1py;

		dN.bottomRowsMul(32u - 8u, 9.0 / 64.0);
	}

	return res;
}

bool b3SDF::interpolate(unsigned int field_id, double& dist, b3Vec3 const& x,
	b3Vec3* gradient) const
{
	if (!m_domain.Contains(x))
		return false;

	b3Vec3 tmpmi = b3MulCW(x - m_domain.lowerBound, m_inv_cell_size);

	unsigned int mi[3] = { (unsigned int)tmpmi[0], (unsigned int)tmpmi[1], (unsigned int)tmpmi[2] };
	if (mi[0] >= m_resolution[0])
		mi[0] = m_resolution[0] - 1;
	if (mi[1] >= m_resolution[1])
		mi[1] = m_resolution[1] - 1;
	if (mi[2] >= m_resolution[2])
		mi[2] = m_resolution[2] - 1;
	b3MultiIndex mui;
	mui[0] = mi[0];
	mui[1] = mi[1];
	mui[2] = mi[2];
	unsigned int i = multiToSingleIndex(mui);
	unsigned int i_ = m_cell_map[field_id][i];
	if (i_ == UINT_MAX)
		return false;

	b3AABB sd = subdomain(i);
	i = i_;
	b3Vec3 d = sd.upperBound - sd.lowerBound;

	b3Vec3 denom = (sd.upperBound - sd.lowerBound);
	b3Vec3 c0 = b3DivCW(b3Vec3(2.0, 2.0, 2.0), denom);
	b3Vec3 c1 = b3DivCW(sd.upperBound + sd.lowerBound, denom);
	b3Vec3 xi = b3MulCW(c0, x) - c1;

	b3Cell32 const& cell = m_cells[field_id][i];
	if (!gradient)
	{
		//auto phi = m_coefficients[field_id][i].dot(shape_function_(xi, 0));
		double phi = 0.0;
		b3ShapeMatrix N = shape_function_(xi, 0);
		for (unsigned int j = 0u; j < 32u; ++j)
		{
			unsigned int v = cell.v[j];
			double c = m_nodes[field_id][v];
			if (c == DBL_MAX)
			{
				return false;
			}
			phi += c * N[j];
		}

		dist = phi;
		return true;
	}

	b3ShapeGradients dN;
	b3ShapeMatrix N = shape_function_(xi, &dN);

	double phi = 0.0;
	gradient->SetZero();
	for (unsigned int j = 0u; j < 32u; ++j)
	{
		unsigned int v = cell.v[j];
		double c = m_nodes[field_id][v];
		if (c == DBL_MAX)
		{
			gradient->SetZero();
			return false;
		}
		phi += c * N[j];
		(*gradient)[0] += c * dN(j, 0);
		(*gradient)[1] += c * dN(j, 1);
		(*gradient)[2] += c * dN(j, 2);
	}
	*gradient = b3MulCW(*gradient, c0);
	dist = phi;
	return true;
}
