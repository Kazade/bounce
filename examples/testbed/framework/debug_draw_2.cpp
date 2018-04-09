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

#include <testbed/framework/debug_draw.h> 

#include <glad_2/glad.h>
#include <imgui/imgui.h>

#include <stdio.h>
#include <stdarg.h>

extern Camera g_camera;
extern DebugDraw* g_debugDraw;
extern const char* g_logName;

static B3_FORCE_INLINE b3Mat34 Convert34(const b3Transform& T)
{
	return b3Mat34(T.rotation.x, T.rotation.y, T.rotation.z, T.position);
}

static B3_FORCE_INLINE b3Mat44 Convert44(const b3Transform& T)
{
	return b3Mat44(
		b3Vec4(T.rotation.x.x, T.rotation.x.y, T.rotation.x.z, 0.0f),
		b3Vec4(T.rotation.y.x, T.rotation.y.y, T.rotation.y.z, 0.0f),
		b3Vec4(T.rotation.z.x, T.rotation.z.y, T.rotation.z.z, 0.0f),
		b3Vec4(T.position.x, T.position.y, T.position.z, 1.0f));
}

b3Mat44 Camera::BuildProjectionMatrix() const
{
	float32 t = tan(0.5f * m_fovy);
	float32 sy = 1.0f / t;

	float32 aspect = m_width / m_height;
	float32 sx = 1.0f / (aspect * t);

	float32 invRange = 1.0f / (m_zNear - m_zFar);
	float32 sz = invRange * (m_zNear + m_zFar);
	float32 tz = invRange *  m_zNear * m_zFar;

	b3Mat44 m;
	m.x = b3Vec4(sx, 0.0f, 0.0f, 0.0f);
	m.y = b3Vec4(0.0f, sy, 0.0f, 0.0f);
	m.z = b3Vec4(0.0f, 0.0f, sz, -1.0f);
	m.w = b3Vec4(0.0f, 0.0f, tz, 0.0f);
	return m;
}

b3Transform Camera::BuildWorldTransform() const
{
	b3Transform xf;
	xf.rotation = b3QuatMat33(m_q);
	xf.position = (m_zoom * xf.rotation.z) - m_center;
	return xf;
}

b3Mat44 Camera::BuildWorldMatrix() const
{
	b3Transform xf = BuildWorldTransform();
	return Convert44(xf);
}

b3Transform Camera::BuildViewTransform() const
{
	b3Transform xf;
	xf.rotation = b3QuatMat33(m_q);
	xf.position = (m_zoom * xf.rotation.z) - m_center;
	return b3Inverse(xf);
}

b3Mat44 Camera::BuildViewMatrix() const
{
	b3Transform xf = BuildViewTransform();
	return Convert44(xf);
}

b3Vec2 Camera::ConvertWorldToScreen(const b3Vec3& pw) const
{
	b3Vec2 ps;
	ps.SetZero();
	return ps;
}

Ray3 Camera::ConvertScreenToWorld(const b3Vec2& ps) const
{
	// Essential Math, page 250.
	float32 t = tan(0.5f * m_fovy);
	float32 aspect = m_width / m_height;

	b3Vec3 pv;
	pv.x = 2.0f * aspect * ps.x / m_width - aspect;
	pv.y = -2.0f * ps.y / m_height + 1.0f;
	pv.z = -1.0f / t;

	b3Transform xf = BuildWorldTransform();
	b3Vec3 pw = xf * pv;

	Ray3 rw;
	rw.direction = b3Normalize(pw - xf.position);
	rw.origin = xf.position;
	rw.fraction = m_zFar;
	return rw;
}

#define BUFFER_OFFSET(i) ((char*)NULL + (i))

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
			Submit();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = color;
		m_sizes[m_count] = size;
		++m_count;
	}

	void Submit()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_camera.BuildViewMatrix();
		b3Mat44 m2 = g_camera.BuildProjectionMatrix();
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
			Submit();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		++m_count;
	}

	void Submit()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_camera.BuildViewMatrix();
		b3Mat44 m2 = g_camera.BuildProjectionMatrix();
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
			Submit();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		m_normals[m_count] = n;
		++m_count;
	}

	void Submit()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		b3Mat44 m1 = g_camera.BuildViewMatrix();
		b3Mat44 m2 = g_camera.BuildProjectionMatrix();
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
		glUseProgram(m_programId);

		b3Mat44 m1 = Convert44(xf);
		m1.x = radius * m1.x;
		m1.y = radius * m1.y;
		m1.z = radius * m1.z;
		b3Mat44 m2 = g_camera.BuildViewMatrix();
		b3Mat44 m3 = g_camera.BuildProjectionMatrix();
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
		glUseProgram(m_programId);

		b3Mat44 m1 = Convert44(xf);
		m1.x = radius * m1.x;
		m1.y = height * m1.y;
		m1.z = radius * m1.z;

		b3Mat44 m2 = g_camera.BuildViewMatrix();
		b3Mat44 m3 = g_camera.BuildProjectionMatrix();
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
		glUseProgram(m_programId);

		b3Mat44 m1 = Convert44(xf);
		m1.x = radius * m1.x;
		m1.y = radius * m1.y;
		m1.z = radius * m1.z;

		b3Mat44 m2 = g_camera.BuildViewMatrix();
		b3Mat44 m3 = g_camera.BuildProjectionMatrix();
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

DebugDraw::DebugDraw()
{
	m_points = new DrawPoints();
	m_lines = new DrawLines();
	m_triangles = new DrawTriangles();
	m_wire = new DrawWire();
	m_solid = new DrawSolid();
}

DebugDraw::~DebugDraw()
{
	delete m_points;
	delete m_lines;
	delete m_triangles;
	delete m_wire;
	delete m_solid;
}

void DebugDraw::DrawPoint(const b3Vec3& p, float32 size, const b3Color& color)
{
	m_points->Vertex(p, size, color);
}

void DebugDraw::DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color)
{
	m_lines->Vertex(p1, color);
	m_lines->Vertex(p2, color);
}

void DebugDraw::DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color)
{
	DrawSegment(p1, p2, color);
	DrawSegment(p2, p3, color);
	DrawSegment(p3, p1, color);
}

void DebugDraw::DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color)
{
	m_triangles->Vertex(p1, color, normal);
	m_triangles->Vertex(p2, color, normal);
	m_triangles->Vertex(p3, color, normal);

	b3Color edgeColor(0.0f, 0.0f, 0.0f, 1.0f);
	DrawTriangle(p2, p3, p3, edgeColor);
}

void DebugDraw::DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Color& color1, const b3Vec3& p2, const b3Color& color2, const b3Vec3& p3, const b3Color& color3)
{
	m_triangles->Vertex(p1, color1, normal);
	m_triangles->Vertex(p2, color2, normal);
	m_triangles->Vertex(p3, color3, normal);

	b3Color edgeColor(0.0f, 0.0f, 0.0f, 1.0f);
	DrawTriangle(p2, p3, p3, edgeColor);
}

void DebugDraw::DrawPolygon(const b3Vec3* vertices, u32 count, const b3Color& color)
{
	b3Vec3 p1 = vertices[count - 1];
	for (u32 i = 0; i < count; ++i)
	{
		b3Vec3 p2 = vertices[i];

		m_lines->Vertex(p1, color);
		m_lines->Vertex(p2, color);

		p1 = p2;
	}
}

void DebugDraw::DrawSolidPolygon(const b3Vec3& normal, const b3Vec3* vertices, u32 count, const b3Color& color)
{
	b3Color fillColor(color.r, color.g, color.b, color.a);

	b3Vec3 p1 = vertices[0];
	for (u32 i = 1; i < count - 1; ++i)
	{
		b3Vec3 p2 = vertices[i];
		b3Vec3 p3 = vertices[i + 1];

		m_triangles->Vertex(p1, fillColor, normal);
		m_triangles->Vertex(p2, fillColor, normal);
		m_triangles->Vertex(p3, fillColor, normal);
	}

	b3Color frameColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 1.0f);
	DrawPolygon(vertices, count, frameColor);
}

void DebugDraw::DrawCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
{
	// Build a tangent vector to normal.
	b3Vec3 u = b3Cross(normal, b3Vec3(1.0f, 0.0f, 0.0f));
	b3Vec3 v = b3Cross(normal, b3Vec3(0.0f, 1.0f, 0.0f));

	// Handle edge cases (zero cross product).
	b3Vec3 n1;
	if (b3LengthSquared(u) > b3LengthSquared(v))
	{
		n1 = u;
	}
	else
	{
		n1 = v;
	}

	n1.Normalize();

	// Build a quaternion to rotate the tangent about the normal.
	u32 kEdgeCount = 20;
	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q(normal, kAngleInc);

	b3Vec3 p1 = center + radius * n1;
	for (u32 i = 0; i < kEdgeCount; ++i)
	{
		b3Vec3 n2 = b3Mul(q, n1);
		b3Vec3 p2 = center + radius * n2;

		m_lines->Vertex(p1, color);
		m_lines->Vertex(p2, color);

		n1 = n2;
		p1 = p2;
	}
}

void DebugDraw::DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
{
	b3Color fillColor(color.r, color.g, color.b, color.a);
	b3Color frameColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 1.0f);

	// Build a tangent vector to normal.
	b3Vec3 u = b3Cross(normal, b3Vec3(1.0f, 0.0f, 0.0f));
	b3Vec3 v = b3Cross(normal, b3Vec3(0.0f, 1.0f, 0.0f));

	// Handle edge cases (zero cross product).
	b3Vec3 n1;
	if (b3LengthSquared(u) > b3LengthSquared(v))
	{
		n1 = u;
	}
	else
	{
		n1 = v;
	}

	n1.Normalize();

	// Build a quaternion to rotate the tangent about the normal.
	const u32 kEdgeCount = 20;
	const float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	b3Quat q(normal, kAngleInc);

	b3Vec3 p1 = center + radius * n1;
	for (u32 i = 0; i < kEdgeCount; ++i)
	{
		b3Vec3 n2 = b3Mul(q, n1);
		b3Vec3 p2 = center + radius * n2;

		m_triangles->Vertex(center, fillColor, normal);
		m_triangles->Vertex(p1, fillColor, normal);
		m_triangles->Vertex(p2, fillColor, normal);

		m_lines->Vertex(p1, frameColor);
		m_lines->Vertex(p2, frameColor);

		n1 = n2;
		p1 = p2;
	}
}

void DebugDraw::DrawSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
	b3Transform xf;
	xf.SetIdentity();
	xf.position = center;

	m_wire->DrawSphere(radius, color, xf);
}

void DebugDraw::DrawSolidSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
	b3Transform xf;
	xf.SetIdentity();
	xf.position = center;

	m_solid->DrawSphere(radius, color, xf);
}

void DebugDraw::DrawCapsule(const b3Vec3& c1, const b3Vec3& c2, float32 radius, const b3Color& color)
{
	float32 height = b3Length(c1 - c2);

	{
		b3Transform xfc;
		xfc.rotation.SetIdentity();
		xfc.position = c1;
		m_wire->DrawSphere(radius, color, xfc);
	}

	if (height > 0.0f)
	{
		DrawSegment(c1, c2, color);
		
		{
			b3Transform xfc;
			xfc.rotation.SetIdentity();
			xfc.position = c2;
			m_wire->DrawSphere(radius, color, xfc);
		}
	}
}

void DebugDraw::DrawSolidCapsule(const b3Vec3& c1, const b3Vec3& c2, float32 radius, const b3Color& c)
{
	float32 height = b3Length(c1 - c2);

	{
		b3Transform xfc;
		xfc.rotation.SetIdentity();
		xfc.position = c1;
		m_solid->DrawSphere(radius, c, xfc);
	}

	if (height > 0.0f)
	{
		{
			b3Mat33 R;
			R.y = (1.0f / height) * (c1 - c2);
			R.z = b3Perp(R.y);
			R.x = b3Cross(R.y, R.z);

			b3Transform xfc;
			xfc.position = 0.5f * (c1 + c2);
			xfc.rotation = R;

			m_solid->DrawCylinder(radius, height, c, xfc);
		}

		{
			b3Transform xfc;
			xfc.rotation.SetIdentity();
			xfc.position = c2;
			m_solid->DrawSphere(radius, c, xfc);
		}
	}
}

void DebugDraw::DrawTransform(const b3Transform& xf)
{
	float32 lenght = 1.0f;

	b3Color red(1.0f, 0.0f, 0.0f, 1.0f);
	b3Color green(0.0f, 1.0f, 0.0f, 1.0f);
	b3Color blue(0.0f, 0.0f, 1.0f, 1.0f);

	b3Vec3 position = xf.position;
	b3Mat33 rotation = xf.rotation;

	b3Vec3 A = position + lenght * rotation.x;
	b3Vec3 B = position + lenght * rotation.y;
	b3Vec3 C = position + lenght * rotation.z;

	DrawSegment(position, A, red);
	DrawSegment(position, B, green);
	DrawSegment(position, C, blue);
}

void DebugDraw::DrawAABB(const b3AABB3& aabb, const b3Color& color)
{
	b3Vec3 lower = aabb.m_lower;
	b3Vec3 upper = aabb.m_upper;

	b3Vec3 vs[8];

	vs[0] = lower;
	vs[1] = b3Vec3(lower.x, upper.y, lower.z);
	vs[2] = b3Vec3(upper.x, upper.y, lower.z);
	vs[3] = b3Vec3(upper.x, lower.y, lower.z);

	vs[4] = upper;
	vs[5] = b3Vec3(upper.x, lower.y, upper.z);
	vs[6] = b3Vec3(lower.x, lower.y, upper.z);
	vs[7] = b3Vec3(lower.x, upper.y, upper.z);

	DrawSegment(vs[0], vs[1], color);
	DrawSegment(vs[1], vs[2], color);
	DrawSegment(vs[2], vs[3], color);
	DrawSegment(vs[3], vs[0], color);

	DrawSegment(vs[4], vs[5], color);
	DrawSegment(vs[5], vs[6], color);
	DrawSegment(vs[6], vs[7], color);
	DrawSegment(vs[7], vs[4], color);

	DrawSegment(vs[2], vs[4], color);
	DrawSegment(vs[5], vs[3], color);

	DrawSegment(vs[6], vs[0], color);
	DrawSegment(vs[1], vs[7], color);
}

void DebugDraw::DrawString(const char* text, const b3Color& color, ...)
{
	va_list arg;
	va_start(arg, text);
	ImGui::Begin(g_logName, NULL, ImVec2(0, 0), 0.0f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, arg);
	ImGui::End();
	va_end(arg);
}

void DebugDraw::DrawSphere(const b3SphereShape* s, const b3Color& c, const b3Transform& xf)
{
	b3Transform xfc;
	xfc.rotation = xf.rotation;
	xfc.position = xf * s->m_center;
	m_solid->DrawSphere(s->m_radius, c, xfc);
}

void DebugDraw::DrawCapsule(const b3CapsuleShape* s, const b3Color& c, const b3Transform& xf)
{
	b3Vec3 c1 = s->m_centers[0];
	b3Vec3 c2 = s->m_centers[1];
	float32 height = b3Length(c1 - c2);
	float32 radius = s->m_radius;

	{
		b3Transform xfc;
		xfc.rotation = xf.rotation;
		xfc.position = xf * c1;
		m_solid->DrawSphere(radius, c, xfc);
	}

	if (height > 0.0f)
	{
		{
			b3Mat33 R;
			R.y = (1.0f / height) * (c1 - c2);
			R.z = b3Perp(R.y);
			R.x = b3Cross(R.y, R.z);

			b3Transform xfc;
			xfc.position = xf * (0.5f * (c1 + c2));
			xfc.rotation = xf.rotation * R;

			m_solid->DrawCylinder(radius, height, c, xfc);
		}

		{
			b3Transform xfc;
			xfc.rotation = xf.rotation;
			xfc.position = xf * c2;
			m_solid->DrawSphere(radius, c, xfc);
		}
	}
}

void DebugDraw::DrawHull(const b3HullShape* s, const b3Color& c, const b3Transform& xf)
{
	const b3Hull* hull = s->m_hull;

	for (u32 i = 0; i < hull->faceCount; ++i)
	{
		const b3Face* face = hull->GetFace(i);
		const b3HalfEdge* begin = hull->GetEdge(face->edge);

		b3Vec3 n = xf.rotation * hull->planes[i].normal;

		const b3HalfEdge* edge = hull->GetEdge(begin->next);
		do
		{
			u32 i1 = begin->origin;
			u32 i2 = edge->origin;
			const b3HalfEdge* next = hull->GetEdge(edge->next);
			u32 i3 = next->origin;

			b3Vec3 p1 = xf * hull->vertices[i1];
			b3Vec3 p2 = xf * hull->vertices[i2];
			b3Vec3 p3 = xf * hull->vertices[i3];

			m_triangles->Vertex(p1, c, n);
			m_triangles->Vertex(p2, c, n);
			m_triangles->Vertex(p3, c, n);

			edge = next;
		} while (hull->GetEdge(edge->next) != begin);
	}
}

void DebugDraw::DrawMesh(const b3MeshShape* s, const b3Color& c, const b3Transform& xf)
{
	const b3Mesh* mesh = s->m_mesh;
	for (u32 i = 0; i < mesh->triangleCount; ++i)
	{
		const b3Triangle* t = mesh->triangles + i;

		b3Vec3 p1 = xf * mesh->vertices[t->v1];
		b3Vec3 p2 = xf * mesh->vertices[t->v2];
		b3Vec3 p3 = xf * mesh->vertices[t->v3];

		b3Vec3 n1 = b3Cross(p2 - p1, p3 - p1);
		n1.Normalize();

		m_triangles->Vertex(p1, c, n1);
		m_triangles->Vertex(p2, c, n1);
		m_triangles->Vertex(p3, c, n1);

		b3Vec3 n2 = -n1;

		m_triangles->Vertex(p1, c, n2);
		m_triangles->Vertex(p3, c, n2);
		m_triangles->Vertex(p2, c, n2);
	}
}

void DebugDraw::DrawShape(const b3Shape* s, const b3Color& c, const b3Transform& xf)
{
	switch (s->GetType())
	{
	case e_sphereShape:
	{
		DrawSphere((b3SphereShape*)s, c, xf);
		break;
	}
	case e_capsuleShape:
	{
		DrawCapsule((b3CapsuleShape*)s, c, xf);
		break;
	}
	case e_hullShape:
	{
		DrawHull((b3HullShape*)s, c, xf);
		break;
	}
	case e_meshShape:
	{
		DrawMesh((b3MeshShape*)s, c, xf);
		break;
	}
	default:
	{
		break;
	}
	}
}

void DebugDraw::Draw(const b3World& world)
{
	for (b3Body* b = world.GetBodyList().m_head; b; b = b->GetNext())
	{
		b3Color c;
		if (b->IsAwake() == false)
		{
			c = b3Color(0.5f, 0.25f, 0.25f, 1.0f);
		}
		else if (b->GetType() == e_staticBody)
		{
			c = b3Color(0.5f, 0.5f, 0.5f, 1.0f);
		}
		else if (b->GetType() == e_dynamicBody)
		{
			c = b3Color(1.0f, 0.5f, 0.5f, 1.0f);
		}
		else
		{
			c = b3Color(0.5f, 0.5f, 1.0f, 1.0f);
		}

		b3Transform xf = b->GetTransform();
		for (b3Shape* s = b->GetShapeList().m_head; s; s = s->GetNext())
		{
			DrawShape(s, c, xf);
		}
	}

	g_debugDraw->Submit();
}

void DebugDraw::Submit()
{
	m_triangles->Submit();
	m_lines->Submit();
	m_points->Submit();
}