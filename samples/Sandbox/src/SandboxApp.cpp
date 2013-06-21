/*
 * Copyright (c) 2013 David Wicks
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
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"
#include "sansumbrella/Box2D.h"
#include "cinder/Triangulate.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace su;

class SandboxApp : public AppNative {
  public:
  void prepareSettings( Settings *settings );
	void setup();
	void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void keyDown( KeyEvent event );
	void update();
	void draw();
private:
  void createCrazyShape();
  void applyForceToShape( const ci::Vec2f &force );
	Sandbox   mSandbox;
  b2Body*   mCrazyBody = nullptr;
};

void SandboxApp::prepareSettings(Settings *settings)
{
  settings->setWindowSize( 1024, 768 );
  settings->enableHighDensityDisplay();
}

void SandboxApp::setup()
{
  // create boundary shapes around the outside edges of the screen
  mSandbox.createBoundaryRect( getWindowBounds() );
  // enable mouse interaction through a b2MouseBody
  mSandbox.connectUserSignals( getWindow() );

  createCrazyShape();
  for( int i = 0; i < 150; ++i )
  {
    Vec2f loc{ Rand::randFloat(getWindowWidth()), getWindowHeight() / 2 + Rand::randFloat(getWindowHeight()/2) };
    mSandbox.createCircle( loc, Rand::randFloat( 5.0f, 25.0f ) );
  }
}

void SandboxApp::keyDown(KeyEvent event)
{
#ifdef CINDER_GLES
// iOS doesn't support key codes
#else
  switch ( event.getCode() )
  {
    case KeyEvent::KEY_c:
      createCrazyShape();
      break;
    case KeyEvent::KEY_UP:
      applyForceToShape( Vec2f{ 0, -1000.0f } );
      break;
    case KeyEvent::KEY_DOWN:
      applyForceToShape( Vec2f{ 0, 1000.0f } );
      break;
    case KeyEvent::KEY_RIGHT:
      applyForceToShape( Vec2f{ 1000.0f, 0.0f } );
      break;
    case KeyEvent::KEY_LEFT:
      applyForceToShape( Vec2f{ -1000.0f, 0.0f } );
      break;
    default:
      break;
  }
#endif
}

void SandboxApp::applyForceToShape(const ci::Vec2f &force)
{
  mCrazyBody->ApplyForceToCenter( b2Vec2{ force.x, force.y } );
//  mCrazyBody->ApplyForce( b2Vec2{ force.x, force.y}, b2Vec2{ mSandbox.toPhysics(getWindowWidth()/2), mSandbox.toPhysics(getWindowHeight()) } );
}

void SandboxApp::mouseDown( MouseEvent event )
{
  if( !event.isAltDown() )
	{
		mSandbox.createCircle( event.getPos(), Rand::randFloat( 5.0f, 20.0f ) );
	}
}

void SandboxApp::mouseDrag(MouseEvent event)
{
  if( !event.isAltDown() )
	{
    mSandbox.createCircle( event.getPos(), Rand::randFloat( 5.0f, 20.0f ) );
	}
}

void SandboxApp::createCrazyShape()
{
  double d1 = getElapsedSeconds();
  float d = Rand::randFloat( 0.5, 2.0f );
  Path2d outline;
  outline.moveTo( cos( 0 ) * d, sin( 0 ) * d );
  const int points = 12;
  for( int i = 1; i < points; ++i )
  {
    float t = lmap<float>( i, 0, points, 0, M_PI * 2 );
    d = Rand::randFloat( 0.5, 2.0f );
    outline.lineTo( cos( t ) * d, sin( t ) * d  );
  }

  vector<b2Vec2> hull_vertices( outline.getNumPoints() );
  for( int i = 0; i < hull_vertices.size(); ++i )
  {
    hull_vertices[i] = b2Vec2{ outline.getPoint(i).x, outline.getPoint(i).y };
  }
  if( mCrazyBody ){ mSandbox.destroyBody( mCrazyBody ); }
  mCrazyBody = mSandbox.createFanShape( mSandbox.toPhysics( getWindowSize() / 2 ), hull_vertices );
//  mCrazyBody = mSandbox.createShape( mSandbox.toPhysics( getWindowSize() / 2 ), Triangulator( outline, 1.0 ).calcMesh( Triangulator::WINDING_ODD ) );
  double d2 = getElapsedSeconds();
  cout << "Creating shape required: " << (d2 - d1) * 1000 << "ms" << endl;
}

void SandboxApp::update()
{
  mSandbox.step();
}

void SandboxApp::draw()
{
	gl::clear( Color::black() );

  mSandbox.debugDraw();
}

CINDER_APP_NATIVE( SandboxApp, RendererGl( RendererGl::AA_NONE ) )
