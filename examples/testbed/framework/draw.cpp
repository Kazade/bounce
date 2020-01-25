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
	scalar w = m_width, h = m_height;
	
	scalar t = tan(0.5f * m_fovy);
	scalar ratio = w / h;
	scalar sx = 1.0f / (ratio * t);
	scalar sy = 1.0f / t;
	
	scalar inv_range = 1.0f / (m_zNear - m_zFar);
	scalar sz = inv_range * (m_zNear + m_zFar);
	
	scalar tz = inv_range * m_zNear * m_zFar;

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
	xf.rotation = m_q;
	xf.translation = (m_zoom * m_q.GetZAxis()) - m_center;
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
	xf.rotation = m_q;
	xf.translation = (m_zoom * m_q.GetZAxis()) - m_center;
	return b3Inverse(xf);
}

b3Mat44 Camera::BuildViewMatrix() const
{
	b3Transform xf = BuildViewTransform();
	return b3TransformMat44(xf);
}

b3Vec2 Camera::ConvertWorldToScreen(const b3Vec3& pw3) const
{
	scalar w = m_width, h = m_height;
	b3Mat44 P = BuildProjectionMatrix();
	b3Mat44 V = BuildViewMatrix();

	b3Vec4 pw(pw3.x, pw3.y, pw3.z, 1.0f);

	b3Vec4 pp = P * V * pw;

	b3Vec3 pn(pp.x, pp.y, pp.z);
	scalar inv_w = pp.w != 0.0f ? 1.0f / pp.w : 1.0f;
	pn *= inv_w;

	scalar u = 0.5f * (pn.x + 1.0f);
	scalar v = 0.5f * (pn.y + 1.0f);

	b3Vec2 ps;
	ps.x = u * w;
	ps.y = (1.0f - v) * h;
	return ps;
}

b3Ray3 Camera::ConvertScreenToWorld(const b3Vec2& ps) const
{
	scalar w = m_width, h = m_height;
	
	scalar t = tan(0.5f * m_fovy);
	scalar ratio = w / h;
	
	b3Vec3 vv;
	vv.x = 2.0f * ratio * ps.x / w - ratio;
	vv.y = -2.0f * ps.y / h + 1.0f;
	vv.z = -1.0f / t;

	b3Transform xf = BuildWorldTransform();
	
	b3Vec3 vw = b3Mul(xf.rotation, vv);
	vw.Normalize();

	b3Ray3 rw;
	rw.direction = vw;
	rw.origin = xf.translation;
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

void Draw::DrawPoint(const b3Vec3& p, scalar size, const b3Color& color)
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

void Draw::DrawCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color)
{
	b3Vec3 n1, n3;
	b3ComputeBasis(normal, n1, n3);

	u32 kEdgeCount = 20;
	scalar kAngleInc = 2.0f * B3_PI / scalar(kEdgeCount);
	b3Quat q;
	q.SetAxisAngle(normal, kAngleInc);

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

void Draw::DrawSolidCircle(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color)
{
	b3Color fillColor(color.r, color.g, color.b, color.a);
	b3Color frameColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 1.0f);

	b3Vec3 n1, n3;
	b3ComputeBasis(normal, n1, n3);

	const u32 kEdgeCount = 20;
	const scalar kAngleInc = 2.0f * B3_PI / scalar(kEdgeCount);
	
	b3Quat q;
	q.SetAxisAngle(normal, kAngleInc);

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

void Draw::DrawSphere(const b3Vec3& center, scalar radius, const b3Color& color)
{
	b3Transform xf;
	xf.rotation.SetIdentity();
	xf.translation = center;

	m_wire->DrawSphere(radius, color, xf);
}

void Draw::DrawSolidSphere(const b3Vec3& center, scalar radius, const b3Quat& rotation, const b3Color& color)
{
	b3Transform xf;
	xf.rotation = rotation;
	xf.translation = center;

	m_solid->DrawSphere(radius, color, xf);
}

void Draw::DrawCapsule(const b3Vec3& c1, const b3Vec3& c2, scalar radius, const b3Color& color)
{
	scalar height = b3Length(c1 - c2);

	{
		b3Transform xf;
		xf.rotation.SetIdentity();
		xf.translation = c1;
		m_wire->DrawSphere(radius, color, xf);
	}

	if (height > 0.0f)
	{
		DrawSegment(c1, c2, color);

		{
			b3Transform xf;
			xf.rotation.SetIdentity();
			xf.translation = c2;
			m_wire->DrawSphere(radius, color, xf);
		}
	}
}

void Draw::DrawSolidCapsule(const b3Vec3& c1, const b3Vec3& c2, scalar radius, const b3Quat& rotation, const b3Color& c)
{
	scalar height = b3Length(c1 - c2);

	{
		b3Transform xf;
		xf.rotation = rotation;
		xf.translation = c1;
		m_solid->DrawSphere(radius, c, xf);
	}

	if (height > 0.0f)
	{
		{
			b3Mat33 R;
			R.y = (1.0f / height) * (c1 - c2);
			b3ComputeBasis(R.y, R.z, R.x);

			b3Quat Q = b3Mat33Quat(R);

			b3Transform xf;
			xf.translation = 0.5f * (c1 + c2);
			xf.rotation = Q;

			m_solid->DrawCylinder(radius, height, c, xf);
		}

		{
			b3Transform xf;
			xf.rotation = rotation;
			xf.translation = c2;
			m_solid->DrawSphere(radius, c, xf);
		}
	}
}

void Draw::DrawTransform(const b3Transform& xf)
{
	scalar lenght = 1.0f;

	b3Vec3 translation = xf.translation;
	b3Mat33 rotation = b3QuatMat33(xf.rotation);

	b3Vec3 A = translation + lenght * rotation.x;
	b3Vec3 B = translation + lenght * rotation.y;
	b3Vec3 C = translation + lenght * rotation.z;

	DrawSegment(translation, A, b3Color_red);
	DrawSegment(translation, B, b3Color_green);
	DrawSegment(translation, C, b3Color_blue);
}

void Draw::DrawAABB(const b3AABB& aabb, const b3Color& color)
{
	b3Vec3 lower = aabb.lowerBound;
	b3Vec3 upper = aabb.upperBound;

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

void Draw::DrawPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color)
{
	b3Vec3 n1, n2;
	b3ComputeBasis(normal, n1, n2);

	scalar scale = 2.0f * radius;
	
	// v1__v4
	// |    |
	// v2__v3
	b3Vec3 v1 = center - scale * n1 - scale * n2;
	b3Vec3 v2 = center + scale * n1 - scale * n2;
	b3Vec3 v3 = center + scale * n1 + scale * n2;
	b3Vec3 v4 = center - scale * n1 + scale * n2;

	DrawSegment(v1, v2, color);
	DrawSegment(v2, v3, color);
	DrawSegment(v3, v4, color);
	DrawSegment(v4, v1, color);
	
	DrawSegment(center, center + normal, color);
}

void Draw::DrawSolidPlane(const b3Vec3& normal, const b3Vec3& center, scalar radius, const b3Color& color)
{
	b3Color frameColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 1.0f);

	b3Vec3 n1, n2;
	b3ComputeBasis(normal, n1, n2);

	scalar scale = 2.0f * radius;

	b3Vec3 v1 = center - scale * n1 - scale * n2;
	b3Vec3 v2 = center + scale * n1 - scale * n2;
	b3Vec3 v3 = center + scale * n1 + scale * n2;
	b3Vec3 v4 = center - scale * n1 + scale * n2;
	
	DrawSegment(v1, v2, frameColor);
	DrawSegment(v2, v3, frameColor);
	DrawSegment(v3, v4, frameColor);
	DrawSegment(v4, v1, frameColor);

	DrawSegment(center, center + normal, frameColor);

	DrawSolidTriangle(normal, v1, v2, v3, color);
	DrawSolidTriangle(normal, v3, v4, v1, color);
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
	
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
	ImGui::Begin("Superlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, args);
	ImGui::End();
	ImGui::PopStyleVar();
	
	va_end(args);
}

void Draw::DrawString(const b3Color& color, const char* text, ...)
{
	va_list args;
	va_start(args, text);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f); 
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), text, args);
	ImGui::End();
	ImGui::PopStyleVar();
	
	va_end(args);
}

void Draw::Flush()
{
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
}