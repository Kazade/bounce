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

#include <glad/glad.h>
#include <GL/glu.h>
#include <stdio.h>
#include <stdarg.h>

#include <imgui/imgui.h>

extern Camera g_camera;

Mat44 Camera::BuildProjectionMatrix() const
{
	// Tangent of the half cone angle along the y-axis
	float32 t = tan(0.5f * m_fovy);
	float32 sy = 1.0f / t;

	// Set the x-scale equals to the y-scale and 
	// proportional to the aspect ratio
	float32 aspect = m_width / m_height;
	float32 sx = 1.0f / (aspect * t);

	float32 invRange = 1.0f / (m_zNear - m_zFar);
	float32 sz = invRange * (m_zNear + m_zFar);
	float32 tz = invRange *  m_zNear * m_zFar;

	Mat44 xf;
	xf.x.Set(sx, 0.0f, 0.0f, 0.0f);
	xf.y.Set(0.0f, sy, 0.0f, 0.0f);
	xf.z.Set(0.0f, 0.0f, sz, -1.0f);
	xf.w.Set(0.0f, 0.0f, tz, 0.0f);

	return xf;
}

b3Transform Camera::BuildWorldTransform() const
{
	b3Transform xf;
	xf.rotation = b3ConvertQuatToRot(m_q);
	xf.position = (m_zoom * xf.rotation.z) - m_center;
	return xf;
}

Mat44 Camera::BuildWorldMatrix() const
{
	b3Transform xf = BuildWorldTransform();
	return GetMat44(xf);
}

b3Transform Camera::BuildViewTransform() const
{
	b3Transform xf;
	xf.rotation = b3ConvertQuatToRot(m_q);
	xf.position = (m_zoom * xf.rotation.z) - m_center;
	return b3Inverse(xf);
}

Mat44 Camera::BuildViewMatrix() const
{
	b3Transform xf = BuildViewTransform();
	return GetMat44(xf);
}

b3Vec2 Camera::ConvertWorldToScreen(const b3Vec3& pw) const
{
	Mat44 xf1 = BuildWorldMatrix();
	Mat44 xf2 = BuildProjectionMatrix();
	Mat44 viewProj = xf2 * xf1;

	Vec4 ph(pw.x, pw.y, pw.z, 1.0f);
	Vec4 ppj = viewProj * ph;

	b3Vec3 pn;
	pn.x = ppj.x / ppj.w;
	pn.y = ppj.y / ppj.w;
	pn.z = ppj.z / ppj.w;

	b3Vec2 ps;
	ps.x = 0.5f * m_width * pn.x + (0.5f * m_width);
	ps.y = -0.5f * m_height * pn.y + (0.5f * m_height);
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

struct DrawPoints
{
	DrawPoints()
	{
		m_count = 0;
	}

	~DrawPoints()
	{
	}

	void Add(const b3Vec3& center, float32 radius, const b3Color& color)
	{
		if (m_count == e_quadCapacity)
		{
			Submit();
		}

		m_quads[m_count].center = center;
		m_quads[m_count].radius = radius;
		m_quads[m_count].color = color;
		++m_count;
	}

	void Submit()
	{
		// Build local quads
		b3Vec3 kVertices[6];
		kVertices[0].Set(-1.0f, 1.0f, 0.0f);
		kVertices[1].Set(-1.0f, -1.0f, 0.0f);
		kVertices[2].Set(1.0f, 1.0f, 0.0f);
		kVertices[3].Set(1.0f, 1.0f, 0.0f);
		kVertices[4].Set(-1.0f, -1.0f, 0.0f);
		kVertices[5].Set(1.0f, -1.0f, 0.0f);

		Mat44 xf2 = g_camera.BuildProjectionMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixf(&xf2.x.x);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		b3Transform xf1 = g_camera.BuildViewTransform();
		for (u32 i = 0; i < m_count; ++i)
		{
			const Quad* c = m_quads + i;
			b3Color color = c->color;

			// Put the center of the quads
			// into the reference frame of the camera
			// so they are rendered in the xy plane
			b3Vec3 v1 = c->radius * kVertices[0] + xf1 * c->center;
			b3Vec3 v2 = c->radius * kVertices[1] + xf1 * c->center;
			b3Vec3 v3 = c->radius * kVertices[2] + xf1 * c->center;
			b3Vec3 v4 = c->radius * kVertices[3] + xf1 * c->center;
			b3Vec3 v5 = c->radius * kVertices[4] + xf1 * c->center;
			b3Vec3 v6 = c->radius * kVertices[5] + xf1 * c->center;

			glBegin(GL_TRIANGLES);
			glColor4f(color.r, color.g, color.b, color.a);
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glEnd();

			glBegin(GL_TRIANGLES);
			glColor4f(color.r, color.g, color.b, color.a);
			glVertex3f(v4.x, v4.y, v4.z);
			glVertex3f(v5.x, v5.y, v5.z);
			glVertex3f(v6.x, v6.y, v6.z);
			glEnd();
		}

		m_count = 0;
	}

	struct Quad
	{
		b3Vec3 center;
		float32 radius;
		b3Color color;
	};

	enum
	{
		e_quadCapacity = 512,
		e_vertexCapacity = 6 * e_quadCapacity
	};

	Quad m_quads[e_quadCapacity];
	u32 m_count;
};

struct DrawLines
{
	DrawLines()
	{
		m_count = 0;
	}

	~DrawLines()
	{
	}

	void Add(const b3Vec3& A, const b3Vec3& B, const b3Color& color)
	{
		if (m_count == e_lineCapacity)
		{
			Submit();
		}

		m_lines[m_count].p = A;
		m_lines[m_count].q = B;
		m_lines[m_count].c = color;
		++m_count;
	}

	void Submit()
	{
		Mat44 xf2 = g_camera.BuildProjectionMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixf(&xf2.x.x);

		Mat44 xf1 = g_camera.BuildViewMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(&xf1.x.x);

		for (u32 i = 0; i < m_count; ++i)
		{
			b3Vec3 p = m_lines[i].p;
			b3Vec3 q = m_lines[i].q;
			b3Color c = m_lines[i].c;

			glBegin(GL_LINES);
			glColor4f(c.r, c.g, c.b, c.a);
			glVertex3f(p.x, p.y, p.z);
			glVertex3f(q.x, q.y, q.z);
			glEnd();
		}

		m_count = 0;
	}

	enum
	{
		e_lineCapacity = 512,
		e_vertexCapacity = 2 * e_lineCapacity
	};

	struct Line
	{
		b3Vec3 p;
		b3Vec3 q;
		b3Color c;
	};

	Line m_lines[e_vertexCapacity];
	u32 m_count;
};

struct DrawTriangles
{
	DrawTriangles()
	{
		m_count = 0;
	}

	~DrawTriangles()
	{
	}

	void Add(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Color& color)
	{
		if (m_count == e_triangleCapacity)
		{
			Submit();
		}

		m_triangles[m_count].col = color;
		m_triangles[m_count].a = A;
		m_triangles[m_count].b = B;
		m_triangles[m_count].c = C;

		++m_count;
	}

	void Submit()
	{
		Mat44 xf2 = g_camera.BuildProjectionMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixf(&xf2.x.x);

		Mat44 xf1 = g_camera.BuildViewMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(&xf1.x.x);

		for (u32 i = 0; i < m_count; ++i)
		{
			b3Vec3 a = m_triangles[i].a;
			b3Vec3 b = m_triangles[i].b;
			b3Vec3 c = m_triangles[i].c;
			b3Color col = m_triangles[i].col;

			glBegin(GL_TRIANGLES);
			glColor4f(col.r, col.g, col.b, col.a);
			glVertex3f(a.x, a.y, a.z);
			glVertex3f(b.x, b.y, b.z);
			glVertex3f(c.x, c.y, c.z);
			glEnd();
		}

		m_count = 0;
	}

	struct Triangle
	{
		b3Color col;
		b3Vec3 a, b, c;
	};

	enum
	{
		e_triangleCapacity = 512,
		e_vertexCapacity = 3 * e_triangleCapacity
	};

	Triangle m_triangles[e_triangleCapacity];
	u32 m_count;
};

struct DrawShapes
{
	DrawShapes()
	{
		m_sphere = gluNewQuadric();
		m_cylinder = gluNewQuadric();
	}

	~DrawShapes()
	{
		gluDeleteQuadric(m_sphere);
		gluDeleteQuadric(m_cylinder);
	}

	void DrawSphere(const b3SphereShape* s, const b3Transform& xf)
	{
		float32 radius = s->m_radius;
		Mat44 xf4 = GetMat44(xf);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glMultMatrixf(&xf4.x.x);
		gluSphere(m_sphere, radius, 10, 10);
		glPopMatrix();
	}

	void DrawCapsule(const b3CapsuleShape* s, const b3Transform& xf)
	{
		b3Vec3 c1 = s->m_centers[0];
		b3Vec3 c2 = s->m_centers[1];
		float32 radius = s->m_radius;
		float32 height = b3Length(c1 - c2);
		b3Vec3 n1 = (1.0f / height) * (c1 - c2);
		b3Vec3 n2 = -n1;

		{
			b3Transform xfc;
			xfc.rotation = xf.rotation;
			xfc.position = xf * c1;

			Mat44 m = GetMat44(xfc);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glMultMatrixf(&m.x.x);

			GLdouble plane[4];
			plane[0] = n1.x;
			plane[1] = n1.y;
			plane[2] = n1.z;
			plane[3] = 0.05f;
			glClipPlane(GL_CLIP_PLANE0, plane);

			glEnable(GL_CLIP_PLANE0);
			gluSphere(m_sphere, radius, 10, 10);
			glDisable(GL_CLIP_PLANE0);

			glPopMatrix();
		}

		{
			b3Transform xfc;
			xfc.rotation = xf.rotation;
			xfc.position = xf * c2;

			Mat44 m = GetMat44(xfc);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glMultMatrixf(&m.x.x);

			GLdouble plane[4];
			plane[0] = n2.x;
			plane[1] = n2.y;
			plane[2] = n2.z;
			plane[3] = 0.05f;
			glClipPlane(GL_CLIP_PLANE0, plane);

			glEnable(GL_CLIP_PLANE0);
			gluSphere(m_sphere, radius, 10, 10);
			glDisable(GL_CLIP_PLANE0);

			glPopMatrix();
		}

		{
			Mat44 m = GetMat44(xf);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glMultMatrixf(&m.x.x);
			glTranslatef(0.0f, 0.5f * height, 0.0f);
			glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
			gluCylinder(m_cylinder, 0.96f * radius, 0.96f * radius, height, 20, 10);
			glPopMatrix();
		}
	}

	void DrawHull(const b3HullShape* s, const b3Transform& xf)
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

				b3Vec3 v1 = xf * hull->vertices[i1];
				b3Vec3 v2 = xf * hull->vertices[i2];
				b3Vec3 v3 = xf * hull->vertices[i3];

				glBegin(GL_TRIANGLES);
				glNormal3f(n.x, n.y, n.z);

				glVertex3f(v1.x, v1.y, v1.z);
				glVertex3f(v2.x, v2.y, v2.z);
				glVertex3f(v3.x, v3.y, v3.z);

				glEnd();

				edge = next;
			} while (hull->GetEdge(edge->next) != begin);

		}
	}

	void DrawMesh(const b3MeshShape* s, const b3Transform& xf)
	{
		const b3Mesh* mesh = s->m_mesh;
		
		for (u32 i = 0; i < mesh->triangleCount; ++i)
		{
			const b3Triangle* t = mesh->triangles + i;

			b3Vec3 v1 = xf * mesh->vertices[t->v1];
			b3Vec3 v2 = xf * mesh->vertices[t->v2];
			b3Vec3 v3 = xf * mesh->vertices[t->v3];

			b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
			n.Normalize();

			glBegin(GL_TRIANGLES);
			glNormal3f(n.x, n.y, n.z);

			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);

			glEnd();
		}
	}

	void DrawShape(const b3Shape* s, const b3Transform& xf)
	{
		switch (s->GetType())
		{
		case e_sphereShape:
		{
			DrawSphere((b3SphereShape*)s, xf);
			break;
		}
		case e_capsuleShape:
		{
			DrawCapsule((b3CapsuleShape*)s, xf);
			break;
		}
		case e_hullShape:
		{
			DrawHull((b3HullShape*)s, xf);
			break;
		}
		case e_meshShape:
		{
			DrawMesh((b3MeshShape*)s, xf);
			break;
		}
		default:
		{
			break;
		}
		}
	}

	void Draw(const b3World& world)
	{
		Mat44 xf1 = g_camera.BuildViewMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(&xf1.x.x);

		Mat44 xf2 = g_camera.BuildProjectionMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixf(&xf2.x.x);

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		float light_ambient[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
		float light_diffuse[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
		float light_position[4] = { 0.0f, 0.0f, 1.0f, 0.0f };
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);

		b3Body* b = world.GetBodyList().m_head;
		while (b)
		{
			b3Color fillColor;
			if (b->IsAwake() == false)
			{
				fillColor = b3Color(0.5f, 0.25f, 0.25f, 1.0f);
			}
			else if (b->GetType() == e_staticBody)
			{
				fillColor = b3Color(0.5f, 0.5f, 0.5f, 1.0f);
			}
			else if (b->GetType() == e_dynamicBody)
			{
				fillColor = b3Color(1.0f, 0.5f, 0.5f, 1.0f);
			}
			else
			{
				fillColor = b3Color(0.5f, 0.5f, 1.0f, 1.0f);
			}

			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glColor4fv(&fillColor.r);

			const b3Transform& xf = b->GetTransform();
			b3Shape* s = b->GetShapeList().m_head;
			while (s)
			{
				DrawShape(s, xf);
				s = s->GetNext();
			}

			glDisable(GL_COLOR_MATERIAL);

			b = b->GetNext();
		}

		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
	}

	GLUquadricObj* m_sphere;
	GLUquadricObj* m_cylinder;
};

DebugDraw::DebugDraw()
{
	m_points = new DrawPoints();
	m_lines = new DrawLines();
	m_triangles = new DrawTriangles();
	m_shapes = new DrawShapes();
}

DebugDraw::~DebugDraw()
{
	delete m_points;
	delete m_lines;
	delete m_triangles;
	delete m_shapes;
}

void DebugDraw::DrawPoint(const b3Vec3& p, const b3Color& color)
{
	m_points->Add(p, 0.05f, color);
}

void DebugDraw::DrawSegment(const b3Vec3& a, const b3Vec3& b, const b3Color& color)
{
	m_lines->Add(a, b, color);
}

void DebugDraw::DrawPolygon(const b3Vec3* vertices, u32 count, const b3Color& color)
{
	b3Vec3 p1 = vertices[count - 1];
	for (u32 i = 0; i < count; ++i)
	{
		b3Vec3 p2 = vertices[i];
		m_lines->Add(p1, p2, color);
		p1 = p2;
	}
}

void DebugDraw::DrawSolidPolygon(const b3Vec3* vertices, u32 count, const b3Color& color)
{
	b3Vec3 p1 = vertices[0];
	for (u32 i = 1; i < count - 1; ++i)
	{
		b3Vec3 p2 = vertices[i];
		b3Vec3 p3 = vertices[i + 1];
		m_triangles->Add(p1, p2, p3, color);
	}

	b3Color frameColor(0.0f, 0.0f, 0.0f, 1.0f);
	DrawPolygon(vertices, count, frameColor);
}

void DebugDraw::DrawCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
{
	u32 kEdgeCount = 20;
	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	float32 cosInc = cos(kAngleInc);
	float32 sinInc = sin(kAngleInc);
	float32 tInc = 1.0f - cosInc;

	b3Vec3 n1 = b3Perp(normal);
	b3Vec3 v1 = center + radius * n1;
	for (u32 i = 0; i < kEdgeCount; ++i)
	{
		// Rodrigues' rotation formula
		b3Vec3 n2 = cosInc * n1 + sinInc * b3Cross(normal, n1) + tInc * b3Dot(normal, n1) * normal;
		b3Vec3 v2 = center + radius * n2;

		m_lines->Add(v1, v2, color);

		n1 = n2;
		v1 = v2;
	}
}

void DebugDraw::DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
{
	u32 kEdgeCount = 20;
	float32 kAngleInc = 2.0f * B3_PI / float32(kEdgeCount);
	float32 cosInc = cos(kAngleInc);
	float32 sinInc = sin(kAngleInc);
	float32 tInc = 1.0f - cosInc;

	b3Vec3 n1 = b3Perp(normal);
	b3Vec3 v1 = center + radius * n1;
	for (u32 i = 0; i < kEdgeCount; ++i)
	{
		// Rodrigues' rotation formula
		b3Vec3 n2 = cosInc * n1 + sinInc * b3Cross(normal, n1) + tInc * b3Dot(normal, n1) * normal;
		b3Vec3 v2 = center + radius * n2;

		m_triangles->Add(center, v1, v2, color);

		n1 = n2;
		v1 = v2;
	}

	b3Color frameColor(0.0f, 0.0f, 0.0f);
	DrawCircle(normal, center, radius, frameColor);
}

void DebugDraw::DrawSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
}

void DebugDraw::DrawSolidSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
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

	// Face 1
	vs[0] = lower;
	vs[1] = b3Vec3(lower.x, upper.y, lower.z);
	vs[2] = b3Vec3(upper.x, upper.y, lower.z);
	vs[3] = b3Vec3(upper.x, lower.y, lower.z);

	// Face 2
	vs[4] = upper;
	vs[5] = b3Vec3(upper.x, lower.y, upper.z);
	vs[6] = b3Vec3(lower.x, lower.y, upper.z);
	vs[7] = b3Vec3(lower.x, upper.y, upper.z);

	// Face 1 edges
	DrawSegment(vs[0], vs[1], color);
	DrawSegment(vs[1], vs[2], color);
	DrawSegment(vs[2], vs[3], color);
	DrawSegment(vs[3], vs[0], color);

	// Face 2 edges
	DrawSegment(vs[4], vs[5], color);
	DrawSegment(vs[5], vs[6], color);
	DrawSegment(vs[6], vs[7], color);
	DrawSegment(vs[7], vs[4], color);

	// Face 3 edges
	DrawSegment(vs[2], vs[4], color);
	DrawSegment(vs[5], vs[3], color);

	// Face 4 edges
	DrawSegment(vs[6], vs[0], color);
	DrawSegment(vs[1], vs[7], color);
}

void DebugDraw::DrawString(const char* text, const b3Color& color, ...)
{
	va_list arg;
	va_start(arg, text);
	ImGui::Begin("Log", NULL, ImVec2(0, 0), 0.0f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, arg);
	ImGui::End();
	va_end(arg);
}

void DebugDraw::Submit(const b3World& world)
{
	m_shapes->Draw(world);
}

void DebugDraw::Submit()
{
	m_triangles->Submit();
	m_lines->Submit();
	m_points->Submit();
}
