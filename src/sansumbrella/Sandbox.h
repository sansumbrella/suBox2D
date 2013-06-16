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

namespace sansumbrella
{
  class Sandbox
  {
  public:
    Sandbox() = default;
    ~Sandbox() = default;
    // run the physics timestep
    void step();
    // remove all bodies
    void clear();

    // have a look at what's in the physics system (scaled up to screen space)
    void debugDraw( bool drawBodies=true, bool drawContacts=true );

    // initialize the box2d world, optionally create boundaries at edges of screen
    void init( bool useScreenBounds=true );
    // create custom boundary
    b2Body* createBoundaryRect( ci::Rectf screen_bounds, float thickness=1.0f );

    // add a BoxElement
    b2Body* createBox( ci::Vec2f pos, ci::Vec2f size );
    b2Body* createBody( const b2BodyDef &body_def, const std::vector<b2FixtureDef> &fixture_defs );
    b2Body* createBody( const b2BodyDef &body_def, const b2FixtureDef &fixture_def );

    // wrappers for some b2world functions
    int32 getBodyCount(){ return mWorld.GetBodyCount(); }
    int32 getContactCount(){ return mWorld.GetContactCount(); }
    b2Body* getBodyList(){ return mWorld.GetBodyList(); }

    // some useful settings
    void setGravity( ci::Vec2f gravity );
    void setVelocityIterations( int vi ){ mVelocityIterations = vi; }
    void setPositionIterations( int pi ){ mPositionIterations = pi; }
    void setTimeStep( float hz ){ mTimeStep = hz; }

    // get the world
    b2World& getWorld(){ return mWorld; }

    // set the filter function for your objects
    void setContactFilter( b2ContactFilter filter );

    // enable user interaction (needs to know what window the interaction is coming from)
    void connectUserSignals( ci::app::WindowRef window );
    void disconnectUserSignals();

    // handle user interaction
    bool mouseDown( ci::app::MouseEvent &event );
    bool mouseUp( ci::app::MouseEvent &event );
    bool mouseDrag( ci::app::MouseEvent &event );

  private:
    int mVelocityIterations = 8;
    int mPositionIterations = 3;
    float mTimeStep = 1.0f / 60.0f;

    b2Vec2          mGravity = b2Vec2( 0, 10.0f );
    // the box2d world
    b2World         mWorld = b2World( mGravity );
    // optional contact filter (kept here to ensure it stays in scope)
    // set this if you want to control what collides with what
    b2ContactFilter mContactFilter;

    // storing
    float           mBoundaryDepth = 5.0f;
    // our mouse, for simple interaction
    b2MouseJoint*   mMouseJoint = nullptr;
    b2Body*         mMouseBody = nullptr;
  };
}