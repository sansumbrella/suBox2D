/*
 * Copyright (c) 2013 David Wicks, sansumbrella.com
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

#include "Renderer.h"
#include "cinder/gl/gl.h"

using namespace box2d;
using namespace cinder;
using namespace std;

Renderer::Renderer()
{
  updateFlags();
}

Renderer::~Renderer()
{}

void Renderer::updateFlags()
{
  SetFlags( (drawShape * e_shapeBit) |
           (drawJoint * e_jointBit) |
           (drawAABB * e_aabbBit) |
           (drawPairs * e_pairBit) |
           (drawCenterOfMass * e_centerOfMassBit ) );
}

void Renderer::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  Path2d path;
  path.moveTo( vertices[0].x, vertices[0].y );
  for( int i = 1; i < vertexCount; ++i )
  {
    path.lineTo( vertices[i].x, vertices[i].y );
  }
  path.close();

  gl::color( color.r, color.g, color.b );
  gl::draw( path );
}

void Renderer::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  Path2d path;
  path.moveTo( vertices[0].x, vertices[0].y );
  for( int i = 1; i < vertexCount; ++i )
  {
    path.lineTo( vertices[i].x, vertices[i].y );
  }
  path.close();

  gl::enableAlphaBlending();
  gl::color( color.r, color.g, color.b, 0.5f );
  gl::drawSolid( path );
  gl::disableAlphaBlending();
  gl::draw( path );
}

void Renderer::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
{
  gl::color( color.r, color.g, color.b );
  gl::drawStrokedCircle( Vec2f{center.x, center.y}, radius );
}

void Renderer::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
{
  gl::enableAlphaBlending();
  gl::color( color.r, color.g, color.b, 0.5f );
  gl::drawSolidCircle( Vec2f{ center.x, center.y }, radius, 16 );
  gl::disableAlphaBlending();
  gl::drawStrokedCircle( Vec2f{center.x, center.y}, radius, 16 );
}

void Renderer::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
  gl::color(color.r, color.g, color.b);
  gl::drawLine( Vec2f{ p1.x, p1.y }, Vec2f{ p2.x, p2.y } );
}

void Renderer::DrawTransform(const b2Transform& xf)
{
	b2Vec2 p1 = xf.p, p2;
	const float32 k_axisScale = 0.4f;

	gl::color(1.0f, 0.0f, 0.0f);
	p2 = p1 + k_axisScale * xf.q.GetXAxis();
	gl::drawLine( Vec2f{ p1.x, p1.y }, Vec2f{ p2.x, p2.y } );

	gl::color(0.0f, 1.0f, 0.0f);
	p2 = p1 + k_axisScale * xf.q.GetYAxis();
	gl::drawLine( Vec2f{ p1.x, p1.y }, Vec2f{ p2.x, p2.y } );
}

void Renderer::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color)
{
	glLineWidth(size);
	gl::color(color.r, color.g, color.b);
	gl::drawSolidCircle( Vec2f( p.x, p.y ), size * 0.5f );
	glLineWidth(1.0f);
}

void Renderer::DrawString(int x, int y, const char* string, ...)
{
	std::cout << "WARNING: " <<  __PRETTY_FUNCTION__ << " Not Yet Implemented" << std::endl;
}

void Renderer::DrawAABB(b2AABB* aabb, const b2Color& color)
{
	gl::color(color.r, color.g, color.b);
  gl::drawStrokedRect( Rectf{ aabb->upperBound.x, aabb->upperBound.y, aabb->lowerBound.x, aabb->lowerBound.y }  );
}
