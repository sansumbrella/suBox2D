/*
 * Copyright (c) 2010—2013, David Wicks, sansumbrella.com
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

#include "Sandbox.h"
#include "cinder/TriMesh.h"
#include <array>

using namespace cinder;
using namespace box2d;
using namespace std;

Sandbox::Sandbox()
{
  mWorld.SetDebugDraw( &mDebugRenderer );
  mContactListener.listenToWorld( &mWorld );
}

void Sandbox::step()
{
	mWorld.Step(mTimeStep, mVelocityIterations, mPositionIterations);
  mWorld.ClearForces();
}

void Sandbox::setContactFilter( const b2ContactFilter &filter )
{
	mContactFilter = filter;
	mWorld.SetContactFilter(&mContactFilter);
}

void Sandbox::debugDraw( float points_per_meter )
{
	gl::pushModelMatrix();
	gl::scale( points_per_meter, points_per_meter );
  mWorld.DrawDebugData();
  gl::popModelMatrix();
}

unique_body_ptr Sandbox::createBody(const b2BodyDef &body_def, const b2FixtureDef &fixture_def)
{
  b2Body *body = mWorld.CreateBody( &body_def );
  body->CreateFixture( &fixture_def );
  return manage( body );
}

unique_body_ptr Sandbox::createBody(const b2BodyDef &body_def, const std::vector<b2FixtureDef> &fixture_defs)
{
  b2Body *body = mWorld.CreateBody( &body_def );
  for( auto &def : fixture_defs )
  {
    body->CreateFixture( &def );
  }
  return manage( body );
}

unique_body_ptr Sandbox::createBody(const b2BodyDef &body_def)
{
  return manage( mWorld.CreateBody( &body_def ) );
}

unique_body_ptr Sandbox::createBox( const ci::vec2 &pos, const ci::vec2 &size, float rotation )
{
  b2BodyDef bodyDef;
	bodyDef.position.Set(	pos.x,
                       pos.y );
  bodyDef.angle = rotation;
	bodyDef.type = b2_dynamicBody;

	b2PolygonShape box;
	box.SetAsBox(	size.x,
                size.y );

	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &box;
	bodyFixtureDef.density = 1.0f;
	bodyFixtureDef.friction = 0.3f;

  return createBody( bodyDef, bodyFixtureDef );
}

unique_body_ptr Sandbox::createCircle( const vec2 &pos, float radius )
{
  b2BodyDef bodyDef;
  bodyDef.position.Set(	pos.x, pos.y );
	bodyDef.type = b2_dynamicBody;

  b2CircleShape circle;
  circle.m_radius = radius;
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circle;
  fixtureDef.density = 1.0f;
  fixtureDef.friction = 0.3f;

  return createBody( bodyDef, fixtureDef );
}

unique_body_ptr Sandbox::createFanShape(const ci::vec2 &pos, const std::vector<b2Vec2> &hull_vertices)
{
  assert( hull_vertices.size() >= 3 );
  vector<b2PolygonShape> shapes( hull_vertices.size() );
  vector<b2FixtureDef> fixtures( hull_vertices.size() );
  b2BodyDef bodyDef;
  bodyDef.position.Set( pos.x, pos.y );
  bodyDef.type = b2_dynamicBody;

  b2Vec2 center{ 0, 0 };

  for( int i = 0; i < hull_vertices.size() - 1; ++i )
  {
    array<b2Vec2, 3> vertices = { center, hull_vertices[i], hull_vertices[i+1] };
    shapes.at( i ).Set( &vertices[0], vertices.size() );
    fixtures.at( i ).shape = &shapes.at( i );
    fixtures[i].density = 1.0f;
    fixtures[i].friction = 0.3f;
  }

  { // connect around loop
    array<b2Vec2, 3> vertices = { center, hull_vertices[hull_vertices.size() - 1], hull_vertices[0] };
    shapes.back().Set( &vertices[0], vertices.size() );
    fixtures.back().shape = &shapes.back();
    fixtures.back().density = 1.0f;
    fixtures.back().friction = 0.3f;
  }

  return createBody( bodyDef, fixtures );
}

unique_body_ptr Sandbox::createShape( const ci::vec2 &centroid, const ci::TriMesh &mesh, float scale )
{
  const auto num_triangles = mesh.getNumTriangles();
  vector<b2PolygonShape> shapes( num_triangles );
  vector<b2FixtureDef> fixtures;
  b2BodyDef bodyDef;
  bodyDef.position.Set( centroid.x, centroid.y );
  bodyDef.type = b2_dynamicBody;

  for( auto i = 0; i < num_triangles; ++i )
  {
    vec2 a, b, c;
    mesh.getTriangleVertices( i, &a, &b, &c );
    if( scale != 1.0f )
    { // resize the triangles if necessary
      a *= scale;
      b *= scale;
      c *= scale;
    }
    // Since we don't know anything about the quality of triangles from the mesh
    // Check that the triangle is wound CCW (has positive area)
    float area = (vec2{b-a}).cross( vec2{c-b} ) / 2;
    array<b2Vec2, 3> vertices = { b2Vec2{a.x, a.y}, b2Vec2{b.x, b.y}, b2Vec2{c.x, c.y} };
    if( area < 0 )
    { // flip the vertex order to be CCW
      vertices = { b2Vec2{a.x, a.y}, b2Vec2{c.x, c.y}, b2Vec2{b.x, b.y} };
    }
    if( abs(area) > b2_linearSlop * 0.5 )
    { // if the triangle is big enough for Box2D to consider
      shapes[i].Set( &vertices[0], vertices.size() );

      b2FixtureDef fixture;
      fixture.shape = &shapes[i];
      fixture.density = 1.0f;
      fixture.friction = 0.3f;
      fixtures.push_back( fixture );
    }
  }
  return createBody( bodyDef, fixtures );
}

unique_joint_ptr Sandbox::createJoint(const b2JointDef &joint_def)
{
  return manage( mWorld.CreateJoint( &joint_def ) );
}

void Sandbox::createBoundaryRect(ci::Rectf screen_bounds)
{
  // half width and half height
  const float w = screen_bounds.getWidth() / 2.0f;
  const float h = screen_bounds.getHeight() / 2.0f;
  // center x, y
  const vec2 upperLeft = screen_bounds.getUpperLeft();
  const float x = upperLeft.x + w;
  const float y = upperLeft.y + h;

  b2BodyDef bodyDef;
  bodyDef.position.Set( x, y );
  bodyDef.type = b2_staticBody;

  array<b2Vec2, 4> vertices = { b2Vec2{-w, -h},
    { w, -h },
    { w, h },
    { -w, h }
  };

  b2ChainShape chain;
  chain.CreateLoop( &vertices[0], vertices.size() );

  b2FixtureDef fixture;
  fixture.shape = &chain;

  mBoundaryBody = createBody( bodyDef, fixture );
}
