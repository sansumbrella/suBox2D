/*
 * Copyright (c) 2013 David Wicks
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Box2DRenderer.h"
#include "cinder/gl/gl.h"

using namespace sansumbrella;
using namespace cinder;
using namespace std;

Box2DRenderer::Box2DRenderer( float points_per_meter ):
mPointsPerMeter( points_per_meter )
{
  updateFlags();
}

Box2DRenderer::~Box2DRenderer()
{}

void Box2DRenderer::updateFlags()
{
  SetFlags( (drawShape * e_shapeBit) |
           (drawJoint * e_jointBit) |
           (drawAABB * e_aabbBit) |
           (drawPairs * e_pairBit) |
           (drawCenterOfMass * e_centerOfMassBit ) );
}

void Box2DRenderer::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f( vertices[i].x, vertices[i].y );
	}
	glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
  gl::enableAlphaBlending();
  glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
  glBegin(GL_TRIANGLE_FAN);
  for (int32 i = 0; i < vertexCount; ++i)
  {
    glVertex2f( vertices[i].x, vertices[i].y );
  }
  glEnd();
  gl::disableAlphaBlending();

  //  glColor4f(color.r, color.g, color.b, 1.0f);
  gl::color( Color::white() );
  glBegin(GL_LINE_LOOP);
  for (int32 i = 0; i < vertexCount; ++i)
  {
    glVertex2f( vertices[i].x, vertices[i].y );
  }
  glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
	const float32 k_segments = 16.0f;
	const float32 k_increment = 2.0f * b2_pi / k_segments;
	float32 theta = 0.0f;
	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f( v.x, v.y );
		theta += k_increment;
	}
	glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
	const float32 k_segments = 16.0f;
	const float32 k_increment = 2.0f * b2_pi / k_segments;
	float32 theta = 0.0f;
	gl::enableAlphaBlending();
	glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f( v.x, v.y );
		theta += k_increment;
	}
	glEnd();
	gl::disableAlphaBlending();

	theta = 0.0f;
	glColor4f(color.r, color.g, color.b, 1.0f);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f( v.x, v.y );
		theta += k_increment;
	}
	glEnd();

	b2Vec2 p = center + radius * axis;
	glBegin(GL_LINES);
	glVertex2f(center.x, center.y);
	glVertex2f(p.x, p.y);
	glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
  glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINES);
	glVertex2f(p1.x, p1.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawTransform(const b2Transform& xf)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
	b2Vec2 p1 = xf.p, p2;
	const float32 k_axisScale = 0.4f;
	glBegin(GL_LINES);

	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex2f(p1.x, p1.y);
	p2 = p1 + k_axisScale * xf.q.GetXAxis();
	glVertex2f(p2.x, p2.y);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex2f(p1.x, p1.y);
	p2 = p1 + k_axisScale * xf.q.GetYAxis();
	glVertex2f(p2.x, p2.y);

	glEnd();
  gl::popModelView();
}
void Box2DRenderer::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );
	glPointSize(size);
	glBegin(GL_POINTS);
	glColor3f(color.r, color.g, color.b);
	glVertex2f(p.x, p.y);
	glEnd();
	glPointSize(1.0f);
  gl::popModelView();
}
void Box2DRenderer::DrawString(int x, int y, const char* string, ...)
{
	std::cout << "WARNING: " <<  __PRETTY_FUNCTION__ << " Not Yet Implemented" << std::endl;
}
void Box2DRenderer::DrawAABB(b2AABB* aabb, const b2Color& color)
{
  gl::pushModelView();
  gl::scale( mPointsPerMeter, mPointsPerMeter );

	glColor3f(color.r, color.g, color.b);
	glBegin(GL_LINE_LOOP);
	glVertex2f(aabb->lowerBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->lowerBound.y);
	glVertex2f(aabb->upperBound.x, aabb->upperBound.y);
	glVertex2f(aabb->lowerBound.x, aabb->upperBound.y);
	glEnd();

  gl::popModelView();
}
