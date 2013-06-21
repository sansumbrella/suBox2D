/*
 * Copyright (c) 2010—2013, David Wicks
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
using namespace sansumbrella;
using namespace std;

Sandbox::Sandbox()
{
  mWorld.SetDebugDraw( &mDebugRenderer );
}

void Sandbox::step()
{
	mWorld.Step(mTimeStep, mVelocityIterations, mPositionIterations);
}

void Sandbox::setGravity( Vec2f gravity )
{
	mWorld.SetGravity( b2Vec2{ toPhysics( gravity.x ), toPhysics( gravity.y ) } );
  mWorld.ClearForces();
}

void Sandbox::setPointsPerMeter(float points)
{
  mPointsPerMeter = points;
  mMetersPerPoint = 1.0f / mPointsPerMeter;
}

void Sandbox::clear()
{
	//get rid of everything
	b2Body* node = mWorld.GetBodyList();
	while (node)
	{
		b2Body* b = node;
		node = node->GetNext();
		destroyBody(b);
	}

	b2Joint* joint = mWorld.GetJointList();
	while (joint) {
		b2Joint* j = joint;
		joint = joint->GetNext();
		mWorld.DestroyJoint(j);
	}
}

void Sandbox::setContactFilter( const b2ContactFilter &filter )
{
	mContactFilter = filter;
	mWorld.SetContactFilter(&mContactFilter);
}

void Sandbox::connectUserSignals(ci::app::WindowRef window)
{
  window->getSignalMouseDown().connect( [this]( app::MouseEvent &event ){ mouseDown( event ); } );
  window->getSignalMouseUp().connect( [this]( app::MouseEvent &event ){ mouseUp( event ); } );
  window->getSignalMouseDrag().connect( [this]( app::MouseEvent &event ){ mouseDrag( event ); } );
}

void Sandbox::debugDraw()
{
	gl::pushModelView();
	gl::scale( mPointsPerMeter, mPointsPerMeter );
  mWorld.DrawDebugData();
  gl::popModelView();
}

b2Body* Sandbox::createBody(const b2BodyDef &body_def, const b2FixtureDef &fixture_def)
{
  b2Body *body = mWorld.CreateBody( &body_def );
  body->CreateFixture( &fixture_def );
  return body;
}

b2Body* Sandbox::createBody(const b2BodyDef &body_def, const std::vector<b2FixtureDef> &fixture_defs)
{
  b2Body *body = mWorld.CreateBody( &body_def );
  for( auto &def : fixture_defs )
  {
    body->CreateFixture( &def );
  }
  return body;
}

b2Body* Sandbox::createBox( const ci::Vec2f &pos, const ci::Vec2f &size )
{
  b2BodyDef bodyDef;
	bodyDef.position.Set(	toPhysics(pos.x),
                       toPhysics( pos.y ) );
	bodyDef.type = b2_dynamicBody;

	b2PolygonShape box;
	box.SetAsBox(	toPhysics(size.x),
               toPhysics( size.y ) );

	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &box;
	bodyFixtureDef.density = 1.0f;
	bodyFixtureDef.friction = 0.3f;

  return createBody( bodyDef, bodyFixtureDef );
}

b2Body* Sandbox::createCircle( const Vec2f &pos, float radius )
{
  b2BodyDef bodyDef;
  bodyDef.position.Set(	toPhysics(pos.x),
                       toPhysics( pos.y ) );
	bodyDef.type = b2_dynamicBody;

  b2CircleShape circle;
  circle.m_radius = toPhysics( radius );
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circle;
  fixtureDef.density = 1.0f;
  fixtureDef.friction = 0.3f;

  return createBody( bodyDef, fixtureDef );
}

b2Body* Sandbox::createFanShape(const ci::Vec2f &centroid, const std::vector<b2Vec2> &hull_vertices)
{
  assert( hull_vertices.size() >= 3 );
  vector<b2PolygonShape> shapes( hull_vertices.size() );
  vector<b2FixtureDef> fixtures( hull_vertices.size() );
  b2BodyDef bodyDef;
  bodyDef.position.Set( centroid.x, centroid.y );
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

b2Body* Sandbox::createShape( const ci::Vec2f &centroid, const ci::TriMesh2d &mesh )
{
  const auto num_triangles = mesh.getNumTriangles();
  vector<b2PolygonShape> shapes( num_triangles );
  vector<b2FixtureDef> fixtures( num_triangles );
  b2BodyDef bodyDef;
  bodyDef.position.Set( centroid.x, centroid.y );
  bodyDef.type = b2_dynamicBody;

  for( auto i = 0; i < num_triangles; ++i )
  {
    Vec2f a, b, c;
    // do these come out in any specific order? CW, CCW?
    mesh.getTriangleVertices( i, &a, &b, &c );
    array<b2Vec2, 3> vertices = { b2Vec2{a.x, a.y}, b2Vec2{b.x, b.y}, b2Vec2{c.x, c.y} };
    shapes[i].Set( &vertices[0], vertices.size() );
    fixtures[i].shape = &shapes[i];
    fixtures[i].density = 1.0f;
    fixtures[i].friction = 0.3f;
  }
  return createBody( bodyDef, fixtures );
}

b2Body* Sandbox::createBoundaryRect(ci::Rectf screen_bounds, float thickness)
{
  // half width and half height
  const float w = toPhysics( screen_bounds.getWidth() / 2.0f ) + thickness;
  const float h = toPhysics( screen_bounds.getHeight() / 2.0f ) + thickness;
  // center x, y
  const Vec2f upperLeft = toPhysics( screen_bounds.getUpperLeft() );
  const float x = upperLeft.x + w - thickness;
  const float y = upperLeft.y + h - thickness;

  b2BodyDef bodyDef;
  bodyDef.position.Set( x, y );
  bodyDef.type = b2_staticBody;

  // Left
  b2FixtureDef left;
  b2PolygonShape leftShape;
  leftShape.SetAsBox( thickness, h, b2Vec2( -w, 0 ), 0 );
  left.shape = &leftShape;

  // Right
  b2FixtureDef right;
  b2PolygonShape rightShape;
  rightShape.SetAsBox( thickness, h, b2Vec2( w, 0 ), 0 );
  right.shape = &rightShape;

  // Top
  b2FixtureDef top;
  b2PolygonShape topShape;
  topShape.SetAsBox( w, thickness, b2Vec2( 0, -h ), 0 );
  top.shape = &topShape;

  // Bottom
  b2FixtureDef bottom;
  b2PolygonShape bottomShape;
  bottomShape.SetAsBox( w, thickness, b2Vec2( 0, h ), 0 );
  bottom.shape = &bottomShape;

  if( mBoundaryBody )
  {
    destroyBody( mBoundaryBody );
  }
  mBoundaryBody = createBody( bodyDef, { left, right, top, bottom } );
  return mBoundaryBody;
}

//
//	Mouse interaction
//

// QueryCallback taken from box2d testbed
class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

bool Sandbox::mouseDown( app::MouseEvent &event )
{
	if (mMouseJoint)
	{
		return false;
	}
	// Make a small box around the click point.
	b2Vec2 p{ toPhysics( event.getPos().x ), toPhysics( event.getPos().y ) };
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	mWorld.QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
    cout << "Got a fixture" << endl;
		b2Body* body = callback.m_fixture->GetBody();
		body->SetAwake(true);
		b2MouseJointDef md;
		md.bodyA = mBoundaryBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 2000.0f * body->GetMass();
		mMouseJoint = (b2MouseJoint*)mWorld.CreateJoint(&md);
	}

	return false;
}

bool Sandbox::mouseUp( app::MouseEvent &event )
{
	if (mMouseJoint)
	{
		mWorld.DestroyJoint(mMouseJoint);
		mMouseJoint = nullptr;
	}
	return false;
}

bool Sandbox::mouseDrag( app::MouseEvent &event )
{
	if(mMouseJoint){
		mMouseJoint->SetTarget( b2Vec2{ toPhysics(event.getPos().x), toPhysics(event.getPos().y) } );
	}
	return false;
}