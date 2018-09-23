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

#include <testbed/framework/draw.h> 

#if defined (U_OPENGL_2)
#include <testbed/framework/draw_gl2.h>
#elif defined (U_OPENGL_4)
#include <testbed/framework/draw_gl4.h>
#else
#endif

#include <imgui/imgui.h>

Camera* g_camera = nullptr;
Draw* g_draw = nullptr;

bool g_glDrawPoints;
bool g_glDrawLines;
bool g_glDrawTriangles;
b3Mat44 g_glViewMatrix;
b3Mat44 g_glProjectionMatrix;

Camera::Camera()
{
	m_center.SetZero();
	m_q.SetIdentity();
	m_width = 1024.0f;
	m_height = 768.0f;
	m_zNear = 1.0f;
	m_zFar = 1000.0f;
	m_fovy = 0.25f * B3_PI;
	m_zoom = 10.0f;
}

b3Mat44 Camera::BuildProjectionMatrix() const
{
	float32 w = m_width, h = m_height;
	
	float32 t = tan(0.5f * m_fovy);
	float32 ratio = w / h;
	float32 sx = 1.0f / (ratio * t);
	float32 sy = 1.0f / t;
	
	float32 inv_range = 1.0f / (m_zNear - m_zFar);
	float32 sz = inv_range * (m_zNear + m_zFar);
	
	float32 tz = inv_range * m_zNear * m_zFar;

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
	return b3TransformMat44(xf);
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
	return b3TransformMat44(xf);
}

b3Vec2 Camera::ConvertWorldToScreen(const b3Vec3& pw3) const
{
	float32 w = m_width, h = m_height;
	b3Mat44 P = BuildProjectionMatrix();
	b3Mat44 V = BuildViewMatrix();

	b3Vec4 pw(pw3.x, pw3.y, pw3.z, 1.0f);

	b3Vec4 pp = P * V * pw;

	b3Vec3 pn(pp.x, pp.y, pp.z);
	float32 inv_w = pp.w != 0.0f ? 1.0f / pp.w : 1.0f;
	pn *= inv_w;

	float32 u = 0.5f * (pn.x + 1.0f);
	float32 v = 0.5f * (pn.y + 1.0f);

	b3Vec2 ps;
	ps.x = u * w;
	ps.y = (1.0f - v) * h;
	return ps;
}

b3Ray3 Camera::ConvertScreenToWorld(const b3Vec2& ps) const
{
	float32 w = m_width, h = m_height;
	
	float32 t = tan(0.5f * m_fovy);
	float32 ratio = w / h;
	
	b3Vec3 vv;
	vv.x = 2.0f * ratio * ps.x / w - ratio;
	vv.y = -2.0f * ps.y / h + 1.0f;
	vv.z = -1.0f / t;

	b3Transform xf = BuildWorldTransform();
	
	b3Vec3 vw = xf.rotation * vv;
	vw.Normalize();

	b3Ray3 rw;
	rw.direction = vw;
	rw.origin = xf.position;
	rw.fraction = m_zFar;
	return rw;
}

Draw::Draw()
{
	g_glViewMatrix.SetZero();
	g_glProjectionMatrix.SetZero();
	g_glDrawPoints = true;
	g_glDrawLines = true;
	g_glDrawTriangles = true;

	m_points = new DrawPoints();
	m_lines = new DrawLines();
	m_triangles = new DrawTriangles();
	m_wire = new DrawWire();
	m_solid = new DrawSolid();
}

Draw::~Draw()
{
	delete m_points;
	delete m_lines;
	delete m_triangles;
	delete m_wire;
	delete m_solid;
}

void Draw::SetViewMatrix(const b3Mat44& m)
{
	g_glViewMatrix = m;
}

void Draw::SetProjectionMatrix(const b3Mat44& m)
{
	g_glProjectionMatrix = m;
}

void Draw::EnableDrawPoints(bool flag)
{
	g_glDrawPoints = flag;
}

void Draw::EnableDrawLines(bool flag)
{
	g_glDrawLines = flag;
}

void Draw::EnableDrawTriangles(bool flag)
{
	g_glDrawTriangles = flag;
}

void Draw::DrawPoint(const b3Vec3& p, float32 size, const b3Color& color)
{
	m_points->Vertex(p, size, color);
}

void Draw::DrawSegment(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color)
{
	m_lines->Vertex(p1, color);
	m_lines->Vertex(p2, color);
}

void Draw::DrawTriangle(const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color)
{
	DrawSegment(p1, p2, color);
	DrawSegment(p2, p3, color);
	DrawSegment(p3, p1, color);
}

void Draw::DrawSolidTriangle(const b3Vec3& normal, const b3Vec3& p1, const b3Vec3& p2, const b3Vec3& p3, const b3Color& color)
{
	m_triangles->Vertex(p1, color, normal);
	m_triangles->Vertex(p2, color, normal);
	m_triangles->Vertex(p3, color, normal);
}

void Draw::DrawPolygon(const b3Vec3* vertices, u32 count, const b3Color& color)
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

void Draw::DrawSolidPolygon(const b3Vec3& normal, const b3Vec3* vertices, u32 count, const b3Color& color)
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
}

void Draw::DrawCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
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

void Draw::DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, float32 radius, const b3Color& color)
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

		n1 = n2;
		p1 = p2;
	}
}

void Draw::DrawSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
	b3Transform xf;
	xf.SetIdentity();
	xf.position = center;

	m_wire->DrawSphere(radius, color, xf);
}

void Draw::DrawSolidSphere(const b3Vec3& center, float32 radius, const b3Color& color)
{
	b3Transform xf;
	xf.SetIdentity();
	xf.position = center;

	m_solid->DrawSphere(radius, color, xf);
}

void Draw::DrawCapsule(const b3Vec3& c1, const b3Vec3& c2, float32 radius, const b3Color& color)
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

void Draw::DrawSolidCapsule(const b3Vec3& c1, const b3Vec3& c2, float32 radius, const b3Color& c)
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

void Draw::DrawTransform(const b3Transform& xf)
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

void Draw::DrawAABB(const b3AABB3& aabb, const b3Color& color)
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

void Draw::DrawString(const b3Color& color, const b3Vec2& ps, const char* text, ...)
{
	va_list args;
	va_start(args, text);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width, g_camera->m_height));
	ImGui::Begin("Superlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, args);
	ImGui::End();
	va_end(args);
}

void Draw::DrawString(const b3Color& color, const b3Vec3& pw, const char* text, ...)
{
	b3Vec2 ps = g_camera->ConvertWorldToScreen(pw);

	va_list args;
	va_start(args, text);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width, g_camera->m_height));
	ImGui::Begin("Superlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, args);
	ImGui::End();
	va_end(args);
}

void Draw::DrawString(const b3Color& color, const char* text, ...)
{
	va_list args;
	va_start(args, text);

	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(g_camera->m_width, g_camera->m_height));
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, args);
	ImGui::End();

	va_end(args);
}

void Draw::DrawSolidSphere(const b3SphereShape* s, const b3Color& c, const b3Transform& xf)
{
	b3Transform xfc;
	xfc.rotation = xf.rotation;
	xfc.position = xf * s->m_center;
	m_solid->DrawSphere(s->m_radius, c, xfc);
}

void Draw::DrawSolidCapsule(const b3CapsuleShape* s, const b3Color& c, const b3Transform& xf)
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

void Draw::DrawSolidHull(const b3HullShape* s, const b3Color& c, const b3Transform& xf)
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

void Draw::DrawSolidMesh(const b3MeshShape* s, const b3Color& c, const b3Transform& xf)
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

void Draw::DrawSolidShape(const b3Shape* s, const b3Color& c, const b3Transform& xf)
{
	switch (s->GetType())
	{
	case e_sphereShape:
	{
		DrawSolidSphere((b3SphereShape*)s, c, xf);
		break;
	}
	case e_capsuleShape:
	{
		DrawSolidCapsule((b3CapsuleShape*)s, c, xf);
		break;
	}
	case e_hullShape:
	{
		DrawSolidHull((b3HullShape*)s, c, xf);
		break;
	}
	case e_meshShape:
	{
		DrawSolidMesh((b3MeshShape*)s, c, xf);
		break;
	}
	default:
	{
		break;
	}
	}
}

void Draw::DrawSolidShapes(const b3World& world)
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
			DrawSolidShape(s, c, xf);
		}
	}
}

void Draw::Flush()
{
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
}