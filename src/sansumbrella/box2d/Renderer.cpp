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
#include "cinder/gl/Context.h"
#include "cinder/gl/Vao.h"
#include "cinder/gl/Vbo.h"

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
  SetFlags( (_draw_shape * e_shapeBit) |
           (_draw_joint * e_jointBit) |
           (_draw_aabb * e_aabbBit) |
           (_draw_pair * e_pairBit) |
           (_draw_center_of_mass * e_centerOfMassBit) );
}

void Renderer::flush()
{
  auto &context = *gl::context();
  const auto &shader = context.getGlslProg();
  const auto &vao = context.getDefaultVao();
  const auto &vbo = context.getDefaultArrayVbo(sizeof(Vertex) * glm::max(_triangle_vertices.size(), _line_vertices.size()));

  gl::ScopedVao         vao_scope(vao);
  gl::ScopedBuffer      vbo_scope(vbo);
  gl::ScopedBlendAlpha  blend;

  vao->replacementBindBegin();
  auto position_loc = shader->getAttribSemanticLocation(geom::POSITION);
  auto color_loc = shader->getAttribSemanticLocation(geom::COLOR);
  gl::enableVertexAttribArray(position_loc);
  gl::vertexAttribPointer(position_loc, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, position));
  gl::enableVertexAttribArray(color_loc);
  gl::vertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, color));
  vao->replacementBindEnd();
  context.setDefaultShaderVars();

  {
    // Draw Polygons
    vbo->bufferSubData(0, sizeof(Vertex) * _triangle_vertices.size(), _triangle_vertices.data());

    context.drawArrays(GL_TRIANGLES, 0, _triangle_vertices.size());
  }

  {
    // Draw Lines
    vbo->bufferSubData(0, sizeof(Vertex) * _line_vertices.size(), _line_vertices.data());

    context.drawArrays(GL_LINES, 0, _line_vertices.size());
  }

  _line_vertices.clear();
  _triangle_vertices.clear();
}

void Renderer::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  ColorA c = ColorA(color.r, color.g, color.b);
  vec2 p1 = vec2(vertices[vertexCount - 1].x, vertices[vertexCount - 1].y);
  for (int i = 0; i < vertexCount; ++i)
  {
    vec2 p2 = vec2(vertices[i].x, vertices[i].y);
    _line_vertices.push_back(Vertex{p1, c});
    _line_vertices.push_back(Vertex{p2, c});
    p1 = p2;
  }
}

void Renderer::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
  ColorA c = ColorA(color.r, color.g, color.b) * 0.5f;

  vec2 p0 = vec2(vertices[0].x, vertices[0].y);

  for (int i = 1; i < vertexCount - 1; ++i)
  {
    auto pi = vec2(vertices[i].x, vertices[i].y);
    auto pi1 = vec2(vertices[i + 1].x, vertices[i + 1].y);
    _triangle_vertices.push_back(Vertex{p0, c});
    _triangle_vertices.push_back(Vertex{pi, c});
    _triangle_vertices.push_back(Vertex{pi1, c});
  }

  DrawPolygon(vertices, vertexCount, color);
}

void Renderer::DrawCircle(const b2Vec2& b2_center, float32 radius, const b2Color& b2_color)
{
  const float segments = 16;
  const float increment = 2.0f * M_PI / segments;

  const float sin_inc = sin(increment);
  const float cos_inc = cos(increment);

  ColorA color(b2_color.r, b2_color.g, b2_color.b);
  vec2 center = vec2(b2_center.x, b2_center.y);
  vec2 r1(1, 0);
  vec2 v1 = center + r1 * radius;
  for (int i = 0; i < segments; ++i)
  {
    vec2 r2 = vec2(cos_inc * r1.x - sin_inc * r1.y, sin_inc * r1.x + cos_inc * r1.y);
    vec2 v2 = center + r2 * radius;

    _line_vertices.push_back(Vertex{v1, color});
    _line_vertices.push_back(Vertex{v2, color});

    r1 = r2;
    v1 = v2;
  }
}

void Renderer::DrawSolidCircle(const b2Vec2& b2_center, float32 radius, const b2Vec2& b2_axis, const b2Color& b2_color)
{
  const float segments = 16;
  const float increment = 2.0f * M_PI / segments;

  const float sin_inc = sin(increment);
  const float cos_inc = cos(increment);

  const auto color = ColorA(b2_color.r, b2_color.g, b2_color.b) * 0.5f;
  const vec2 center = vec2(b2_center.x, b2_center.y);
  vec2 r1(1, 0);
  vec2 v1 = center + r1 * radius;

  // Triangles
  for (int i = 0; i < segments; ++i)
  {
    // r2 is a rotation of r1
    vec2 r2 = vec2(cos_inc * r1.x - sin_inc * r1.y, sin_inc * r1.x + cos_inc * r1.y);
    vec2 v2 = center + r2 * radius;

    _triangle_vertices.push_back(Vertex{center, color});
    _triangle_vertices.push_back(Vertex{v1, color});
    _triangle_vertices.push_back(Vertex{v2, color});

    r1 = r2;
    v1 = v2;
  }

  // Lines
  r1 = vec2(1, 0);
  v1 = center + r1 * radius;
  for (int i = 0; i < segments; ++i)
  {
    // r2 is a rotation of r1
    vec2 r2 = vec2(cos_inc * r1.x - sin_inc * r1.y, sin_inc * r1.x + cos_inc * r1.y);
    vec2 v2 = center + r2 * radius;

    _line_vertices.push_back(Vertex{v1, color});
    _line_vertices.push_back(Vertex{v2, color});

    r1 = r2;
    v1 = v2;
  }
}

void Renderer::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
  ColorA c(color.r, color.g, color.b);
  _line_vertices.push_back(Vertex{vec2(p1.x, p1.y), c});
  _line_vertices.push_back(Vertex{vec2(p2.x, p2.y), c});
}

void Renderer::DrawTransform(const b2Transform& transform)
{
  const float32 scaling = 0.4f;
  auto x = transform.q.GetXAxis();
  auto y = transform.q.GetYAxis();
  vec2 center = vec2(transform.p.x, transform.p.y);
  vec2 right = center + vec2(x.x, x.y) * scaling;
  vec2 up = center + vec2(y.x, y.y) * scaling;

  ColorA red(1, 0, 0);
  ColorA green(0, 1, 0);

  _line_vertices.push_back(Vertex{center, red});
  _line_vertices.push_back(Vertex{right, red});

  _line_vertices.push_back(Vertex{center, green});
  _line_vertices.push_back(Vertex{up, green});
}
