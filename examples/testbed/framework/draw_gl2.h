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

#ifndef DRAW_GL2_H
#define DRAW_GL2_H

#include <glad_2/glad.h>

#include <stdio.h>
#include <stdarg.h>

#include <bounce/common/math/transform.h>
#include <bounce/common/math/mat44.h>
#include <bounce/common/draw.h>

#define BUFFER_OFFSET(i) ((char*)NULL + (i))

extern bool g_glDrawPoints;
extern bool g_glDrawLines;
extern bool g_glDrawTriangles;

extern b3Mat44 g_glViewMatrix;
extern b3Mat44 g_glProjectionMatrix;

static void AssertGL()
{
	GLenum errorCode = glGetError();
	if (errorCode != GL_NO_ERROR)
	{
		fprintf(stderr, "OpenGL error = %d\n", errorCode);
		assert(false);
	}
}

static void PrintLog(GLuint id)
{
	GLint log_length = 0;
	if (glIsShader(id))
	{
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &log_length);
	}
	else if (glIsProgram(id))
	{
		glGetProgramiv(id, GL_INFO_LOG_LENGTH, &log_length);
	}
	else
	{
		fprintf(stderr, "Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(id))
	{
		glGetShaderInfoLog(id, log_length, NULL, log);
	}
	else if (glIsProgram(id))
	{
		glGetProgramInfoLog(id, log_length, NULL, log);
	}

	fprintf(stderr, "%s", log);
	free(log);
}

static GLuint CreateShader(const char* source, GLenum type)
{
	GLuint shaderId = glCreateShader(type);

	const char* sources[] = { source };
	glShaderSource(shaderId, 1, sources, NULL);
	glCompileShader(shaderId);

	GLint status = GL_FALSE;
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		fprintf(stderr, "Error compiling %d shader.\n", type);
		PrintLog(shaderId);
		glDeleteShader(shaderId);
		return 0;
	}

	return shaderId;
}

// 
static GLuint CreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vsId = CreateShader(vs, GL_VERTEX_SHADER);
	GLuint fsId = CreateShader(fs, GL_FRAGMENT_SHADER);
	assert(vsId != 0 && fsId != 0);

	GLuint programId = glCreateProgram();
	glAttachShader(programId, vsId);
	glAttachShader(programId, fsId);
	glLinkProgram(programId);

	glDeleteShader(vsId);
	glDeleteShader(fsId);

	GLint status = GL_FALSE;
	glGetProgramiv(programId, GL_LINK_STATUS, &status);
	assert(status != GL_FALSE);

	return programId;
}

struct DrawPoints
{
	DrawPoints()
	{
		const char* vs = \
			"#version 120\n"
			"uniform mat4 projectionMatrix;\n"
			"attribute vec3 v_position;\n"
			"attribute vec4 v_color;\n"
			"attribute float v_size;\n"
			"varying vec4 f_color;\n"
			"void main()\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
			"   gl_PointSize = v_size;\n"
			"}\n";

		const char* fs = \
			"#version 120\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	gl_FragColor = f_color;\n"
			"}\n";

		m_programId = CreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = glGetAttribLocation(m_programId, "v_position");
		m_colorAttribute = glGetAttribLocation(m_programId, "v_color");
		m_sizeAttribute = glGetAttribLocation(m_programId, "v_size");

		glGenBuffers(3, m_vboIds);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Vec3), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Color), m_colors, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(float32), m_sizes, GL_DYNAMIC_DRAW);

		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		m_count = 0;
	}

	~DrawPoints()
	{
		glDeleteProgram(m_programId);
		glDeleteBuffers(3, m_vboIds);
	}

	void Vertex(const b3Vec3& v, float32 size, const b3Color& color)
	{
		if (m_count == e_vertexCapacity)
		{
			Flush();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = color;
		m_sizes[m_count] = size;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}

		if (!g_glDrawPoints)
		{
			m_count = 0;
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_glViewMatrix;
		b3Mat44 m2 = g_glProjectionMatrix;
		b3Mat44 m = m2 * m1;

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3), m_vertices);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);
		glEnableVertexAttribArray(m_colorAttribute);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float32), m_sizes);
		glEnableVertexAttribArray(m_sizeAttribute);
		glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, m_count);
		glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);

		glDisableVertexAttribArray(m_sizeAttribute);

		glDisableVertexAttribArray(m_colorAttribute);

		glDisableVertexAttribArray(m_vertexAttribute);

		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_vertexCapacity = 1024
	};

	b3Vec3 m_vertices[e_vertexCapacity];
	b3Color m_colors[e_vertexCapacity];
	float32 m_sizes[e_vertexCapacity];
	u32 m_count;

	GLuint m_programId;
	GLuint m_projectionUniform;
	GLuint m_vertexAttribute;
	GLuint m_colorAttribute;
	GLuint m_sizeAttribute;

	GLuint m_vboIds[3];
};

struct DrawLines
{
	DrawLines()
	{
		const char* vs = \
			"#version 120\n"
			"uniform mat4 projectionMatrix;\n"
			"attribute vec3 v_position;\n"
			"attribute vec4 v_color;\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 120\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	gl_FragColor = f_color;\n"
			"}\n";

		m_programId = CreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = glGetAttribLocation(m_programId, "v_position");
		m_colorAttribute = glGetAttribLocation(m_programId, "v_color");

		glGenBuffers(2, m_vboIds);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Vec3), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Color), m_colors, GL_DYNAMIC_DRAW);

		AssertGL();
		
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		m_count = 0;
	}

	~DrawLines()
	{
		glDeleteProgram(m_programId);
		glDeleteBuffers(2, m_vboIds);
	}

	void Vertex(const b3Vec3& v, const b3Color& c)
	{
		if (m_count == e_vertexCapacity)
		{
			Flush();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}

		if (!g_glDrawLines)
		{
			m_count = 0;
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_glViewMatrix;
		b3Mat44 m2 = g_glProjectionMatrix;
		b3Mat44 m = m2 * m1;
		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3), m_vertices);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);
		glEnableVertexAttribArray(m_colorAttribute);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		glDrawArrays(GL_LINES, 0, m_count);

		AssertGL();

		glDisableVertexAttribArray(m_colorAttribute);
		
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_vertexCapacity = 2 * 1024
	};

	b3Vec3 m_vertices[e_vertexCapacity];
	b3Color m_colors[e_vertexCapacity];
	u32 m_count;

	GLuint m_programId;
	GLuint m_projectionUniform;
	GLuint m_vertexAttribute;
	GLuint m_colorAttribute;

	GLuint m_vboIds[2];
};

struct DrawTriangles
{
	DrawTriangles()
	{
		const char* vs = \
			"#version 120\n"
			"uniform mat4 projectionMatrix;\n"
			"attribute vec3 v_position;\n"
			"attribute vec4 v_color;\n"
			"attribute vec3 v_normal;\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	vec3 La = vec3(0.5f, 0.5f, 0.5f);\n"
			"	vec3 Ld = vec3(0.5f, 0.5f, 0.5f);\n"
			"	vec3 L = vec3(0.0f, 0.3f, 0.7f);\n"
			"	vec3 Ma = v_color.xyz;\n"
			"	vec3 Md = v_color.xyz;\n"
			"	vec3 a = La * Ma;\n"
			"	vec3 d = max(dot(v_normal, L), 0.0f) * Ld * Md;\n"
			"	f_color = vec4(a + d, v_color.w);\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 120\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	gl_FragColor = f_color;\n"
			"}\n";

		m_programId = CreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = glGetAttribLocation(m_programId, "v_position");
		m_colorAttribute = glGetAttribLocation(m_programId, "v_color");
		m_normalAttribute = glGetAttribLocation(m_programId, "v_normal");

		glGenBuffers(3, m_vboIds);
		
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Vec3), m_vertices, GL_DYNAMIC_DRAW);
		
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Color), m_colors, GL_DYNAMIC_DRAW);
		
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferData(GL_ARRAY_BUFFER, e_vertexCapacity * sizeof(b3Vec3), m_normals, GL_DYNAMIC_DRAW);
		
		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		m_count = 0;
	}

	~DrawTriangles()
	{
		glDeleteProgram(m_programId);
		glDeleteBuffers(3, m_vboIds);
	}

	void Vertex(const b3Vec3& v, const b3Color& c, const b3Vec3& n)
	{
		if (m_count == e_vertexCapacity)
		{
			Flush();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		m_normals[m_count] = n;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}
		
		if (!g_glDrawTriangles)
		{
			m_count = 0;
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_glViewMatrix;
		b3Mat44 m2 = g_glProjectionMatrix;
		b3Mat44 m = m2 * m1;

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3), m_vertices);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Color), m_colors);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_colorAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b3Vec3), m_normals);
		glVertexAttribPointer(m_normalAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_normalAttribute);

		glDrawArrays(GL_TRIANGLES, 0, m_count);

		AssertGL();

		glDisableVertexAttribArray(m_normalAttribute);

		glDisableVertexAttribArray(m_colorAttribute);

		glDisableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_vertexCapacity = 3 * 512
	};

	b3Vec3 m_vertices[e_vertexCapacity];
	b3Color m_colors[e_vertexCapacity];
	b3Vec3 m_normals[e_vertexCapacity];
	u32 m_count;

	GLuint m_programId;
	GLuint m_projectionUniform;
	GLuint m_vertexAttribute;
	GLuint m_colorAttribute;
	GLuint m_normalAttribute;

	GLuint m_vboIds[3];
};

struct DrawWireSphere
{
	enum
	{
		e_rings = 12,
		e_sectors = 12,
		e_vertexCount = e_rings * e_sectors,
		e_indexCount = (e_rings - 1) * (e_sectors - 1) * 8
	};

	DrawWireSphere()
	{
		float32 R = 1.0f / float32(e_rings - 1);
		float32 S = 1.0f / float32(e_sectors - 1);

		b3Vec3 vs[e_vertexCount];
		b3Vec3 ns[e_vertexCount];
		b3Color cs[e_vertexCount];

		u32 vc = 0;
		for (u32 r = 0; r < e_rings; r++)
		{
			for (u32 s = 0; s < e_sectors; s++)
			{
				float32 y = sin(-0.5f * B3_PI + B3_PI * r * R);
				float32 x = cos(2.0f * B3_PI * s * S) * sin(B3_PI * r * R);
				float32 z = sin(2.0f * B3_PI * s * S) * sin(B3_PI * r * R);

				vs[vc].Set(x, y, z);
				cs[vc] = b3Color(1.0f, 1.0f, 1.0f, 1.0f);
				++vc;
			}
		}

		u32 is[e_indexCount];

		u32 ic = 0;
		for (u32 r = 0; r < e_rings - 1; r++)
		{
			for (u32 s = 0; s < e_sectors - 1; s++)
			{
				u32 i1 = r * e_sectors + s;
				u32 i2 = (r + 1) * e_sectors + s;
				u32 i3 = (r + 1) * e_sectors + (s + 1);
				u32 i4 = r * e_sectors + (s + 1);

				is[ic++] = i1;
				is[ic++] = i2;

				is[ic++] = i2;
				is[ic++] = i3;

				is[ic++] = i3;
				is[ic++] = i4;

				is[ic++] = i4;
				is[ic++] = i1;
			}
		}

		glGenBuffers(1, &m_vboId);
		glGenBuffers(1, &m_iboId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glBufferData(GL_ARRAY_BUFFER, vc * sizeof(b3Vec3), vs, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboId);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, ic * sizeof(u32), is, GL_STATIC_DRAW);

		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	~DrawWireSphere()
	{
		glDeleteBuffers(1, &m_vboId);
		glDeleteBuffers(1, &m_iboId);
	}

	GLuint m_vboId;
	GLuint m_iboId;
};

struct DrawWire
{
	DrawWire()
	{
		const char* vs = \
			"#version 120\n"
			"uniform vec4 color;\n"
			"uniform mat4 projectionMatrix;\n"
			"attribute vec3 v_position;\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 120\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	gl_FragColor = f_color;\n"
			"}\n";

		m_programId = CreateShaderProgram(vs, fs);
		m_colorUniform = glGetUniformLocation(m_programId, "color");
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = glGetAttribLocation(m_programId, "v_position");
	}

	~DrawWire()
	{
		glDeleteProgram(m_programId);
	}

	void DrawSphere(float32 radius, const b3Color& c, const b3Transform& xf)
	{
		if (!g_glDrawLines)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = b3TransformMat44(xf);
		m1.x = radius * m1.x;
		m1.y = radius * m1.y;
		m1.z = radius * m1.z;
		b3Mat44 m2 = g_glViewMatrix;
		b3Mat44 m3 = g_glProjectionMatrix;
		b3Mat44 m = m3 * m2 * m1;

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_sphere.m_vboId);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphere.m_iboId);
		glDrawElements(GL_LINES, m_sphere.e_indexCount, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

		glDisableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
	}

	GLuint m_programId;
	GLuint m_colorUniform;
	GLuint m_projectionUniform;
	GLuint m_vertexAttribute;

	DrawWireSphere m_sphere;
};

struct DrawSolidSphere
{
	enum
	{
		e_rings = 18,
		e_sectors = 18,
		e_vertexCount = e_rings * e_sectors,
		e_indexCount = (e_rings - 1) * (e_sectors - 1) * 6,
		e_faceCount = e_indexCount / 3
	};

	DrawSolidSphere()
	{
		float32 R = 1.0f / float32(e_rings - 1);
		float32 S = 1.0f / float32(e_sectors - 1);

		b3Vec3 vs[e_vertexCount];
		b3Vec3 ns[e_vertexCount];

		u32 vc = 0;
		for (u32 r = 0; r < e_rings; r++)
		{
			for (u32 s = 0; s < e_sectors; s++)
			{
				float32 a1 = 2.0f * B3_PI * float32(s) * S;
				float32 c1 = cos(a1);
				float32 s1 = sin(a1);

				float32 a2 = -0.5f * B3_PI + B3_PI * float32(r) * R;
				float32 s2 = sin(a2);

				float32 a3 = B3_PI * float32(r) * R;
				float32 s3 = sin(a3);

				float32 x = c1 * s3;
				float32 y = s2;
				float32 z = s1 * s3;
				
				b3Vec3 v(x, y, z);
				v.Normalize();

				vs[vc] = v;
				ns[vc] = v;
				++vc;
			}
		}

		u32 is[e_indexCount];

		u32 ic = 0;
		for (u32 r = 0; r < e_rings - 1; r++)
		{
			for (u32 s = 0; s < e_sectors - 1; s++)
			{
				u32 i1 = r * e_sectors + s;
				u32 i2 = (r + 1) * e_sectors + s;
				u32 i3 = (r + 1) * e_sectors + (s + 1);
				u32 i4 = r * e_sectors + (s + 1);

				is[ic++] = i1;
				is[ic++] = i2;
				is[ic++] = i3;

				is[ic++] = i1;
				is[ic++] = i3;
				is[ic++] = i4;
			}
		}

		glGenBuffers(3, m_vboIds);
		glGenBuffers(1, &m_iboId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, vc * sizeof(b3Vec3), vs, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, vc * sizeof(b3Vec3), ns, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboId);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, ic * sizeof(u32), is, GL_STATIC_DRAW);

		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	~DrawSolidSphere()
	{
		glDeleteBuffers(2, m_vboIds);
		glDeleteBuffers(1, &m_iboId);
	}

	GLuint m_vboIds[2];
	GLuint m_iboId;
};

struct DrawSolidCylinder
{
	enum
	{
		e_segments = 64,
		e_vertexCount = e_segments * 6,
	};

	DrawSolidCylinder()
	{
		b3Vec3 vs[e_vertexCount];
		b3Vec3 ns[e_vertexCount];

		u32 vc = 0;
		for (u32 i = 0; i < e_segments; ++i)
		{
			float32 t0 = 2.0f * B3_PI * float32(i) / float32(e_segments);
			float32 t1 = 2.0f * B3_PI * float32(i + 1) / float32(e_segments);

			float32 c0 = cos(t0);
			float32 s0 = sin(t0);

			float32 c1 = cos(t1);
			float32 s1 = sin(t1);

			b3Vec3 v1;
			v1.x = s0;
			v1.y = -0.5f;
			v1.z = c0;
			
			b3Vec3 v2;
			v2.x = s1;
			v2.y = -0.5f;
			v2.z = c1;
			
			b3Vec3 v3;
			v3.x = s1;
			v3.y = 0.5f;
			v3.z = c1;
			
			b3Vec3 v4;
			v4.x = s0;
			v4.y = 0.5f;
			v4.z = c0;
			
			b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
			n.Normalize();

			vs[vc] = v1;
			ns[vc] = n;
			++vc;

			vs[vc] = v2;
			ns[vc] = n;
			++vc;

			vs[vc] = v3;
			ns[vc] = n;
			++vc;

			vs[vc] = v1;
			ns[vc] = n;
			++vc;

			vs[vc] = v3;
			ns[vc] = n;
			++vc;

			vs[vc] = v4;
			ns[vc] = n;
			++vc;
		}

		u32 is[e_vertexCount];

		u32 ic = vc;
		for (u32 i = 0; i < vc; ++i)
		{
			is[i] = i;
		}

		glGenBuffers(2, m_vboIds);
		glGenBuffers(1, &m_iboId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, vc * sizeof(b3Vec3), vs, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, vc * sizeof(b3Vec3), ns, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboId);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, ic * sizeof(u32), is, GL_STATIC_DRAW);

		AssertGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	~DrawSolidCylinder()
	{
		glDeleteBuffers(2, m_vboIds);
		glDeleteBuffers(1, &m_iboId);
	}

	GLuint m_vboIds[2];
	GLuint m_iboId;
};

struct DrawSolid
{
	DrawSolid()
	{
		const char* vs = \
			"#version 120\n"
			"uniform vec4 color;\n"
			"uniform mat4 modelMatrix;\n"
			"uniform mat4 projectionMatrix;\n"
			"attribute vec3 v_position;\n"
			"attribute vec3 v_normal;\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	vec3 f_normal = normalize( ( modelMatrix * vec4(v_normal, 0.0f) ).xyz );\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
			"	vec3 La = vec3(0.5f, 0.5f, 0.5f);\n"
			"	vec3 Ld = vec3(0.5f, 0.5f, 0.5f);\n"
			"	vec3 L = vec3(0.0f, 0.3f, 0.7f);\n"
			"	vec3 Ma = color.xyz;\n"
			"	vec3 Md = color.xyz;\n"
			"	vec3 a = La * Ma;\n"
			"	vec3 d = max(dot(f_normal, L), 0.0f) * Ld * Md;\n"
			"	f_color = vec4(a + d, color.w);\n"
			"}\n";

		const char* fs = \
			"#version 120\n"
			"varying vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	gl_FragColor = f_color;\n"
			"}\n";

		m_programId = CreateShaderProgram(vs, fs);
		m_colorUniform = glGetUniformLocation(m_programId, "color");
		m_modelUniform = glGetUniformLocation(m_programId, "modelMatrix");
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = glGetAttribLocation(m_programId, "v_position");
		m_normalAttribute = glGetAttribLocation(m_programId, "v_normal");
	}

	~DrawSolid()
	{
	}

	void DrawCylinder(float32 radius, float32 height, const b3Color& c, const b3Transform& xf)
	{
		if (!g_glDrawTriangles)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = b3TransformMat44(xf);
		m1.x = radius * m1.x;
		m1.y = height * m1.y;
		m1.z = radius * m1.z;

		b3Mat44 m2 = g_glViewMatrix;
		b3Mat44 m3 = g_glProjectionMatrix;
		b3Mat44 m = m3 * m2 * m1;

		glUniform4fv(m_colorUniform, 1, &c.r);
		glUniformMatrix4fv(m_modelUniform, 1, GL_FALSE, &m1.x.x);
		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_cylinder.m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_cylinder.m_vboIds[1]);
		glVertexAttribPointer(m_normalAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_normalAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cylinder.m_iboId);
		glDrawElements(GL_TRIANGLES, m_cylinder.e_vertexCount, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

		glDisableVertexAttribArray(m_normalAttribute);

		glDisableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
	}

	void DrawSphere(float32 radius, const b3Color& c, const b3Transform& xf)
	{
		if (!g_glDrawTriangles)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = b3TransformMat44(xf);
		m1.x = radius * m1.x;
		m1.y = radius * m1.y;
		m1.z = radius * m1.z;

		b3Mat44 m2 = g_glViewMatrix;
		b3Mat44 m3 = g_glProjectionMatrix;
		b3Mat44 m = m3 * m2 * m1;

		glUniform4fv(m_colorUniform, 1, &c.r);
		glUniformMatrix4fv(m_modelUniform, 1, GL_FALSE, &m1.x.x);
		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, &m.x.x);

		glBindBuffer(GL_ARRAY_BUFFER, m_sphere.m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ARRAY_BUFFER, m_sphere.m_vboIds[1]);
		glVertexAttribPointer(m_normalAttribute, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glEnableVertexAttribArray(m_normalAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphere.m_iboId);
		glDrawElements(GL_TRIANGLES, m_sphere.e_indexCount, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

		glDisableVertexAttribArray(m_normalAttribute);

		glDisableVertexAttribArray(m_vertexAttribute);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
	}

	GLuint m_programId;
	GLuint m_colorUniform;
	GLuint m_modelUniform;
	GLuint m_projectionUniform;
	GLuint m_vertexAttribute;
	GLuint m_normalAttribute;

	DrawSolidSphere m_sphere;
	DrawSolidCylinder m_cylinder;
};

#endif