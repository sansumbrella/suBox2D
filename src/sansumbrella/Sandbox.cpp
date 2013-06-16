/*
 *  Sandbox.cpp
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/7/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#include "Sandbox.h"

using namespace cinder;
using namespace sansumbrella;

void Sandbox::step()
{
	mWorld.Step(mTimeStep, mVelocityIterations, mPositionIterations);
}

void Sandbox::setGravity( Vec2f gravity )
{
	mGravity = Conversions::toPhysics( gravity );
	mWorld.SetGravity(mGravity);
}

void Sandbox::clear()
{
	//get rid of everything
	b2Body* node = mWorld.GetBodyList();
	while (node)
	{
		b2Body* b = node;
		node = node->GetNext();

		mWorld.DestroyBody(b);
	}

	b2Joint* joint = mWorld.GetJointList();
	while (joint) {
		b2Joint* j = joint;
		joint = joint->GetNext();

		mWorld.DestroyJoint(j);
	}
}

void Sandbox::setContactFilter( b2ContactFilter filter )
{
	mContactFilter = filter;
	mWorld.SetContactFilter(&mContactFilter);
}

void Sandbox::connectUserSignals(ci::app::WindowRef window)
{
  window->getSignalMouseDown().connect( [this]( app::MouseEvent &event ){ mouseDown( event ); } );
  window->getSignalMouseUp().connect( [this]( app::MouseEvent &event ){ mouseUp( event ); } );
  window->getSignalMouseDrag().connect( [this]( app::MouseEvent &event ){ mouseDrag( event ); } );

  b2BodyDef bodyDef;
  bodyDef.type = b2_staticBody;
  b2CircleShape shape;
  shape.m_radius = 0.5;
  b2FixtureDef fixtureDef;
  fixtureDef.isSensor = true;
  fixtureDef.shape = &shape;
  mMouseBody = createBody( bodyDef, fixtureDef );
}

void Sandbox::debugDraw( bool drawBodies, bool drawContacts )
{
	// should utilize an extension of b2DebugDraw (will soon)
	//
	if( drawBodies )
	{
		//draw all bodies, contact points, etc

		gl::color( ColorA(1.0f, 0.0f, 0.1f, 0.5f) );

		//draw bodies
		b2Body* bodies = mWorld.GetBodyList();
		while( bodies != NULL )
		{
			b2Vec2 pos = bodies->GetPosition();
			float32 angle = bodies->GetAngle();

			gl::pushMatrices();

			gl::translate( Conversions::toScreen(pos) );
			gl::rotate( Conversions::radiansToDegrees( angle ) );

			//draw the fixtures for this body
			b2Fixture* fixtures = bodies->GetFixtureList();
			while( fixtures != NULL )
			{
				//not sure why the base b2Shape doesn't contain the vertex methods...
				switch (fixtures->GetType()) {
					case b2Shape::e_polygon:
          {
            b2PolygonShape* shape = (b2PolygonShape*)fixtures->GetShape();

            glBegin(GL_POLYGON);

            for( int i=0; i != shape->GetVertexCount(); ++i )
            {
              gl::vertex( Conversions::toScreen( shape->GetVertex(i) ) );
            }

            glEnd();
          }
						break;
					case b2Shape::e_circle:
          {
            b2CircleShape* shape = (b2CircleShape*)fixtures->GetShape();
            gl::drawSolidCircle( Conversions::toScreen( shape->m_p ), Conversions::toScreen( shape->m_radius ) );
          }
						break;

					default:
						break;
				}


				fixtures = fixtures->GetNext();
			}

			gl::popMatrices();

			bodies = bodies->GetNext();
		}
	}

	if( drawContacts )
	{
		//draw contacts
		b2Contact* contacts = mWorld.GetContactList();

		gl::color( ColorA( 0.0f, 0.0f, 1.0f, 0.8f ) );
		glPointSize(3.0f);
		glBegin(GL_POINTS);

		while( contacts != NULL )
		{
			b2WorldManifold m;
			contacts->GetWorldManifold(&m);	//grab the

			for( int i=0; i != b2_maxManifoldPoints; ++i )
			{
				Vec2f p = Conversions::toScreen( m.points[i] );
				gl::vertex( p );
			}

			contacts = contacts->GetNext();
		}
		glEnd();
	}
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

b2Body* Sandbox::createBox(ci::Vec2f pos, ci::Vec2f size)
{
  b2BodyDef bodyDef;
	bodyDef.position.Set(	Conversions::toPhysics(pos.x),
                       Conversions::toPhysics( pos.y ) );
	bodyDef.type = b2_dynamicBody;

	b2PolygonShape box;
	box.SetAsBox(	Conversions::toPhysics(size.x),
               Conversions::toPhysics( size.y ) );

	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &box;
	bodyFixtureDef.density = 1.0f;
	bodyFixtureDef.friction = 0.3f;

  return createBody( bodyDef, bodyFixtureDef );
}

b2Body* Sandbox::createBoundaryRect(ci::Rectf screen_bounds, float thickness)
{
  // half width and half height
  const float w = Conversions::toPhysics( screen_bounds.getWidth() / 2.0f ) + thickness;
  const float h = Conversions::toPhysics( screen_bounds.getHeight() / 2.0f ) + thickness;
  // center x, y
  const b2Vec2 upperLeft = Conversions::toPhysics( screen_bounds.getUpperLeft() );
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

  return createBody( bodyDef, { left, right, top, bottom } );
}

void Sandbox::init( bool useScreenBounds )
{
	if( useScreenBounds ){
		createBoundaryRect( app::getWindowBounds() );
	}
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
	if (mMouseJoint != NULL)
	{
		return false;
	}
	b2Vec2 p = Conversions::toPhysics( event.getPos() );
	// Make a small box.
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
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = body;
		md.bodyB = mMouseBody;
		md.target = p;
		md.maxForce = 1000.0f * body->GetMass();
		mMouseJoint = (b2MouseJoint*)mWorld.CreateJoint(&md);
		body->SetAwake(true);
	}

	return false;
}

bool Sandbox::mouseUp( app::MouseEvent &event )
{
	if (mMouseJoint)
	{
		mWorld.DestroyJoint(mMouseJoint);
		mMouseJoint = NULL;
	}
	return false;
}

bool Sandbox::mouseDrag( app::MouseEvent &event )
{
	if(mMouseJoint){
		mMouseJoint->SetTarget( Conversions::toPhysics(event.getPos()) );
	}
	return false;
}