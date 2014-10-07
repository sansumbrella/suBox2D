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

#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Rand.h"

#include "suBox2D.h"

using namespace ci;
using namespace ci::app;
using namespace std;

/**
 Example demonstrating positioning user elements with physics bodies.
 Uses box2d::SimpleControl for basic interaction with bodies.

 Press any key to create a new set of bubbles.
 */


/// Bubbles store a (smart) pointer to a Box2D body and use it to update their
/// rendered positions every frame.
/// This is a reasonable approach to managing a Box2D body in your own structure.
class Bubble
{
public:
  Bubble( b2::unique_body_ptr &&b, const ci::vec2 &l, float r ):
  mBody( move(b) ),
  mLoc( l ),
  mRadius( r ),
  mColor( CM_HSV, Rand::randFloat(0.0f, 0.16f), 0.9f, 0.9f )
  { // bouncy
    mBody->GetFixtureList()->SetRestitution( 0.6f );
  }
  // we can't copy-construct because of unique_ptr, but we can move-construct
  // necessary to have this ctor for std::vector support
  Bubble( Bubble &&other ):
  mBody( move( other.mBody ) ),
  mLoc( other.mLoc ),
  mRadius( other.mRadius ),
  mColor( other.mColor )
  {}
  // update the bubble position from physics, using the world scale
  void update( float scale )
  {
    mLoc.x = mBody->GetPosition().x * scale;
    mLoc.y = mBody->GetPosition().y * scale;
  }
  // draw a circle with a line at bubble location
  void draw()
  {
    gl::color( mColor );
    gl::pushModelView();
    gl::translate( mLoc );
    gl::drawSolidCircle( vec2( 0 ), mRadius );
    gl::rotate( mBody->GetAngle() );
    gl::color( Color::black() );
    gl::drawLine( vec2{ -mRadius, 0 }, vec2{ mRadius, 0 } );
    gl::popModelView();
  }
private:
  b2::unique_body_ptr mBody;
  ci::vec2           mLoc;
  float               mRadius;
  Color               mColor;
};

class CustomDrawingApp : public AppNative {
public:
  void prepareSettings( Settings *settings );
  void setup();
  void update();
  void draw();

private:
  // Sandbox wraps the b2World
  b2::Sandbox         mSandbox;
  // Scale for mapping between physics and screen coordinates
  b2::Scale           mScale;
  b2::SimpleControl   mControl;
  vector<Bubble>      mBubbles;
};

void CustomDrawingApp::prepareSettings(Settings *settings)
{
#if not defined( CINDER_COCOA_TOUCH )
  settings->setWindowSize( 1024, 768 );
#else
  settings->enableMultiTouch( false );
#endif
}

void CustomDrawingApp::setup()
{
  auto bounds = mScale.toPhysics( Rectf{getWindowBounds()} );
  mSandbox.createBoundaryRect( bounds );
  mControl.connectUserSignals( getWindow(), mSandbox, mScale.getMetersPerPoint() );

  auto createBubbles = [=]() -> void
  {
    mBubbles.clear();
    for( int i = 0; i < 29; ++i )
    {
      vec2 loc{ Rand::randFloat( bounds.getX1(), bounds.getX2() ), Rand::randFloat( bounds.getY1(), bounds.getY2() ) };
      float radius = Rand::randFloat( bounds.getWidth() * 0.01f, bounds.getWidth() * 0.1f );
      mBubbles.emplace_back( Bubble{ mSandbox.createCircle( loc, radius ), mScale.fromPhysics(loc), mScale.fromPhysics(radius) } );
    }
  };

  createBubbles();
  getWindow()->getSignalKeyDown().connect( [=](KeyEvent &event) -> void
                                          {
                                            createBubbles();
                                          } );
}

void CustomDrawingApp::update()
{
  mSandbox.step();
  for( auto &bubble : mBubbles )
  {
    bubble.update( mScale.getPointsPerMeter() );
  }
}

void CustomDrawingApp::draw()
{
  gl::clear( Color::black() );
  for( auto &bubble : mBubbles )
  {
    bubble.draw();
  }
}

CINDER_APP_NATIVE( CustomDrawingApp, RendererGl() )
