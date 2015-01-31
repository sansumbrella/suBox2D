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

#pragma once
#include "Common.h"

namespace box2d
{
  class Renderer : public b2Draw
  {
  public:
    Renderer();
    ~Renderer();

    //
    // b2Draw interface overrides.
    //

    void DrawPolygon (const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;
    void DrawSolidPolygon (const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;
    void DrawCircle (const b2Vec2& center, float32 radius, const b2Color& color) override;
    void DrawSolidCircle (const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) override;
    void DrawSegment (const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override;
    void DrawTransform (const b2Transform& xf) override;

    //
    // Set render flags for Box2D.
    // Decide whether to draw different parts of physics.
    //

    void displayShapes (bool doDraw) { _draw_shape = doDraw; updateFlags(); }
    void displayJoints (bool doDraw) { _draw_joint = doDraw; updateFlags(); }
    void displayAABBs (bool doDraw) { _draw_aabb = doDraw; updateFlags(); }
    void displayPairs (bool doDraw) { _draw_pair = doDraw; updateFlags(); }
    void displayCentersOfMass (bool doDraw) { _draw_center_of_mass = doDraw; updateFlags(); }

    /// Draw using OpenGL and clear out drawing buffers.
    void flush();

  private:
    int _draw_shape = 1;
    int _draw_joint = 1;
    int _draw_aabb = 0;
    int _draw_pair = 0;
    int _draw_center_of_mass = 1;

    void updateFlags();

    struct Vertex
    {
      ci::vec2    position;
      ci::ColorA  color;
    };

    std::vector<Vertex> _triangle_vertices;
    std::vector<Vertex> _line_vertices;
  };
}
