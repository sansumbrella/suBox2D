/*
 *  Sandbox.h
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/7/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#pragma once
#include <Box2D/Box2D.h>
#include "cinder/app/App.h"
#include "cinder/Vector.h"
#include "Conversions.h"
#include "cinder/Rand.h"

#include "PhysicsElement.h"
#include "BoxElement.h"
#include "BoundaryElement.h"

namespace cinder{
	namespace box2d {
		
		class Sandbox {
			
		public:
			Sandbox();
			~Sandbox();
			// run the physics timestep
			void update();
			
			// check our bodies to see whether they should be removed
			// from the simulation and removes them
			void removeDeadElements();
			void clear();
			
			// have a look at what's in the physics system (scaled up to screen space)
			void debugDraw( bool drawBodies=true, bool drawContacts=true );
			// draw all PhysicsElements
			void draw();
			
			// initialize the box2d world, optionally create boundaries at edges of screen
			void init( bool useScreenBounds=true );
			// create custom boundary
			void createBoundaries( Rectf screen_bounds );
			BoundaryElement& getBounds(){ return mBounds; }
			
			// add a BoxElement
			void addBox( Vec2f pos, Vec2f size );
			
			// add a user-created PhysicsElement
			// will be used a user-data for a b2Body
			// will be notified on removal of body
			void addElement( PhysicsElement *b );
			
			// wrappers for some b2world functions
			
			int32 getBodyCount(){ return mWorld->GetBodyCount(); }
			int32 getContactCount(){ return mWorld->GetContactCount(); }
			b2Body* getBodyList(){ return mWorld->GetBodyList(); }
			
			// some useful settings
			
			void setGravity( Vec2f gravity );
			void setVelocityIterations( int vi ){ mVelocityIterations = vi; }
			void setPositionIterations( int pi ){ mPositionIterations = pi; }
			void setTimeStep( float hz ){ mTimeStep = hz; }
			
			// get the world
			b2World* getWorld(){ return mWorld; }
			
			// set the filter function for your objects
			void setContactFilter( b2ContactFilter filter );
			
			// enable user interaction (needs to know what window the interaction is coming from)
      void connectUserSignals( ci::app::WindowRef window );
      void disconnectUserSignals(){}
			
			// handle user interaction
			bool mouseDown( app::MouseEvent event );
			bool mouseUp( app::MouseEvent event );
			bool mouseDrag( app::MouseEvent event );
			
		private:
			bool mDoSleep;
			
			CallbackId mMouseDownId, mMouseUpId, mMouseDragId;
			
			int mVelocityIterations;
			int mPositionIterations;
			float mTimeStep;
			
			b2Vec2 mGravity;
			// the box2d world
			b2World* mWorld;
			// optional contact filter (kept here to ensure it stays in scope)
			// set this if you want to control what collides with what
			b2ContactFilter mContactFilter;
			
			// storing boundaries (only necessary for drawing them later)
			// those set by createBoundaries will be remembered here
			BoundaryElement mBounds;
			
			// storing
			float mBoundaryDepth;
			
			// an empty body
			b2Body* mGroundBody;
			
			// used when adding bodies to mWorld
			b2Body* mTempBody;
			
			// our mouse, for simple interaction
			b2MouseJoint* mMouseJoint;			
		};
	}
}