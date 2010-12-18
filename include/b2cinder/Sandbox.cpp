/*
 *  Sandbox.cpp
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/7/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#include "Sandbox.h"

using namespace cinder::box2d;

Sandbox::Sandbox()
{
	// set defaults
	mDoSleep = true;
	mGravity.Set(0.0f, 10.0f);	//since top-left is (0,0), positive gravity goes down
	mVelocityIterations = 10;
	mPositionIterations = 10;
	
	mTimeStep = 1.0f / 60.0f;
	
	mMouseJoint = NULL;
	mBoundaryDepth = 5.0f;
}

Sandbox::~Sandbox()
{
	// unsure whether it's necessary to be explicit about this
	delete mWorld;
}

void Sandbox::update()
{	
	mWorld->Step(mTimeStep, mVelocityIterations, mPositionIterations);
	mWorld->ClearForces();
}

void Sandbox::setGravity( Vec2f gravity )
{
	mGravity = Conversions::screenToPhysics( gravity );
	mWorld->SetGravity(mGravity);
}

void Sandbox::removeDeadElements()
{
	b2Body* node = mWorld->GetBodyList();
	while (node)
	{
		b2Body* b = node;
		node = node->GetNext();
		
		PhysicsElement* p = (PhysicsElement*)b->GetUserData();
		if( p->isDead() )
		{
			mWorld->DestroyBody(b);
			// maybe have a different syntax for this
			p->destroyBody();
		}
	}
}

void Sandbox::clear()
{
	//get rid of everything
	b2Body* node = mWorld->GetBodyList();
	while (node)
	{
		b2Body* b = node;
		node = node->GetNext();
		
		PhysicsElement* p = (PhysicsElement*)b->GetUserData();
		mWorld->DestroyBody(b);
		if( p ){
			p->destroyBody();
		}
		
	}

	b2Joint* joint = mWorld->GetJointList();
	while (joint) {
		b2Joint* j = joint;
		joint = joint->GetNext();
		
		mWorld->DestroyJoint(j);
	}
	
	
}

void Sandbox::setContactFilter( b2ContactFilter filter )
{
	mContactFilter = filter;
	mWorld->SetContactFilter(&mContactFilter);
}

void Sandbox::enableMouseInteraction( app::App *app, bool enable )
{
	mIsMouseEnabled = enable;
	if( mIsMouseEnabled )
	{
		mMouseDownId = app->registerMouseDown( this, &Sandbox::mouseDown );
		mMouseUpId = app->registerMouseUp( this, &Sandbox::mouseUp );
		mMouseDragId = app->registerMouseDrag( this, &Sandbox::mouseDrag );
	} else {
		app->unregisterMouseDown( mMouseDownId );
		app->unregisterMouseUp( mMouseUpId );
		app->unregisterMouseDrag( mMouseDragId );
	}

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
		b2Body* bodies = mWorld->GetBodyList();
		while( bodies != NULL )
		{
			b2Vec2 pos = bodies->GetPosition();
			float32 angle = bodies->GetAngle();
			
			gl::pushMatrices();
			
			gl::translate( Conversions::physicsToScreen(pos) );
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
								gl::vertex( Conversions::physicsToScreen( shape->GetVertex(i) ) );
							}
							
							glEnd();
						}
						break;
					case b2Shape::e_circle:
						{
							b2CircleShape* shape = (b2CircleShape*)fixtures->GetShape();
							gl::drawSolidCircle( Conversions::physicsToScreen( shape->m_p ), Conversions::physicsToScreen( shape->m_radius ) );
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
		b2Contact* contacts = mWorld->GetContactList();
		
		gl::color( ColorA( 0.0f, 0.0f, 1.0f, 0.8f ) );
		glPointSize(3.0f);
		glBegin(GL_POINTS);
		
		while( contacts != NULL )
		{
			b2WorldManifold m;
			contacts->GetWorldManifold(&m);	//grab the 
			
			for( int i=0; i != b2_maxManifoldPoints; ++i )
			{
				Vec2f p = Conversions::physicsToScreen( m.points[i] );
				gl::vertex( p );
			}
			
			contacts = contacts->GetNext();
		}
		glEnd();
	}
}

void Sandbox::draw()
{
	// get our PhysicsElements through the userData of the bodies
	b2Body* bodies = mWorld->GetBodyList();
	while( bodies != NULL )
	{
		PhysicsElement* p = (PhysicsElement*)bodies->GetUserData();
		if( p != NULL)
			p->draw();
		bodies = bodies->GetNext();
	}
}

void Sandbox::addBox( Vec2f pos, Vec2f size )
{	
	b2BodyDef bodyDef;
	bodyDef.position.Set(	Conversions::screenToPhysics(pos.x),
							Conversions::screenToPhysics( pos.y ) );
	
	bodyDef.type = b2_dynamicBody;
	mTempBody = mWorld->CreateBody(&bodyDef);
	
	b2PolygonShape box;
	box.SetAsBox(	Conversions::screenToPhysics(size.x),
					Conversions::screenToPhysics( size.y ) );
	
	b2FixtureDef bodyFixtureDef;
	bodyFixtureDef.shape = &box;
	bodyFixtureDef.density = 1.0f;
	bodyFixtureDef.friction = 0.3f;
	mTempBody->CreateFixture(&bodyFixtureDef);
	
}

void Sandbox::addElement( PhysicsElement *b )
{
	mTempBody = mWorld->CreateBody( b->getBodyDef() );
	
	// add all fixture defs
	std::list<b2FixtureDef>* defs = b->getFixtureDefs();
	for( std::list<b2FixtureDef>::iterator iter = defs->begin(); iter != defs->end(); ++iter )
	{
		mTempBody->CreateFixture( &*(iter) );
	}
	
	// make a circular reference between PhysicsElement and b2Body
	b->setBody( mTempBody );
}

void Sandbox::createBoundaries( Rectf screenBounds )
{
	// add our boundaries
	mBounds.set( screenBounds );
	addElement( &mBounds );
}

void Sandbox::init( bool useScreenBounds )
{	
	// create our world
	mWorld = new b2World(mGravity, mDoSleep);
	
	if( useScreenBounds ){
		createBoundaries( app::getWindowBounds() );
	}
	
	// Create an empty body for use with a b2MouseJoint
	
	b2BodyDef bodyDef;
	mGroundBody = mWorld->CreateBody(&bodyDef);
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

bool Sandbox::mouseDown( app::MouseEvent event )
{
	if (mMouseJoint != NULL)
	{
		return false;
	}
	b2Vec2 p = Conversions::screenToPhysics( event.getPos() );
	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;
	
	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	mWorld->QueryAABB(&callback, aabb);
	
	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = mGroundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0f * body->GetMass();
		mMouseJoint = (b2MouseJoint*)mWorld->CreateJoint(&md);
		body->SetAwake(true);
	}
	
	return false;
}

bool Sandbox::mouseUp( app::MouseEvent event )
{
	if (mMouseJoint)
	{
		mWorld->DestroyJoint(mMouseJoint);
		mMouseJoint = NULL;
	}
	return false;
}

bool Sandbox::mouseDrag( app::MouseEvent event )
{
	if(mMouseJoint){
		mMouseJoint->SetTarget( Conversions::screenToPhysics(event.getPos()) );
	}
	return false;
}