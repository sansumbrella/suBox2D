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
  void reset();
  void buildBodies();
  void createCrazyShape();
  void createTextShape( const std::string &text, const ci::Vec2f &top_left );
  void applyForceToShape( const ci::Vec2f &force );
	Sandbox               mSandbox;
  su::unique_body_ptr   mSpikyBody;
};

void SandboxApp::prepareSettings(Settings *settings)
{
  settings->setWindowSize( 1024, 768 );
  settings->enableHighDensityDisplay();
}

void SandboxApp::setup()
{
  // enable mouse interaction through a b2MouseBody
//  mSandbox.connectUserSignals( getWindow() );
  buildBodies();
}

void SandboxApp::reset()
{
  // release all smart pointers before iterating through the box2d list
  mSpikyBody.reset();
  // loop through all box2d bodies
  while( mSandbox.getBodyCount() > 0 )
  { // Box2D bodies are stored in a linked list;
    // we destroy the head of the list as long as there are bodies
    mSandbox.destroyBody( mSandbox.getBodyList() );
  }
  buildBodies();
}

void SandboxApp::buildBodies()
{
  // Create boundary shapes around the outside edges of the screen
  mSandbox.createBoundaryRect( getWindowBounds() );
  // Create text from triangulated mesh
  createTextShape( "Collapsed", Vec2f( 200.0f, 100.0f ) );
  // Create a randomized shape like a triangle fan
  createCrazyShape();
  // Create a circle
  Vec2f loc{ Rand::randFloat(getWindowWidth()), getWindowHeight() / 2 + Rand::randFloat(getWindowHeight()/2) };
  float radius = Rand::randFloat( 20.0f, 80.0f );
  mSandbox.createCircle( loc, radius );
  // Create a box
  loc = { Rand::randFloat(getWindowWidth()), getWindowHeight() / 2 + Rand::randFloat(getWindowHeight()/2) };
  Vec2f size{ Rand::randFloat( 20.0f, 100.0f ), Rand::randFloat( 20.0f, 100.0f ) };
  mSandbox.createBox( loc, size );
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
    case KeyEvent::KEY_r:
      reset();
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
  mSpikyBody->ApplyForceToCenter( b2Vec2{ force.x, force.y } );
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
  float d = Rand::randFloat( 0.1, 2.0f );
  Path2d outline;
  outline.moveTo( cos( 0 ) * d, sin( 0 ) * d );
  const int points = 12;
  for( int i = 1; i < points; ++i )
  {
    float t = lmap<float>( i, 0, points, 0, M_PI * 2 );
    d = Rand::randFloat( 0.1, 2.0f );
    outline.lineTo( cos( t ) * d, sin( t ) * d  );
  }

  vector<b2Vec2> hull_vertices( outline.getNumPoints() );
  for( int i = 0; i < hull_vertices.size(); ++i )
  {
    hull_vertices[i] = b2Vec2{ outline.getPoint(i).x, outline.getPoint(i).y };
  }
  // create shape as a fan (works well with the radial vertices we defined)
  // much faster than generic triangulation, but only works on certain shapes
  // will crash if the triangles have negative (clockwise) area
  mSpikyBody = mSandbox.manage( mSandbox.createFanShape( mSandbox.toPhysics( getWindowSize() / 2 ), hull_vertices ) );
}

void SandboxApp::createTextShape( const std::string &text, const ci::Vec2f &top_left )
{
  // create shape using Cinder's Triangulator to calculate the triangles
  // works on almost any arbitrary shape
  Font f{ "Arial", 128.0f };
  Vec2f loc{ top_left };
  // de-res the text before getting the physics outline
  // this scalar is just to make it much smaller when we triangulate
  float scalar = 10000.0f;
  for( auto &c : text )
  {
    auto shape = f.getGlyphShape( f.getGlyphChar( c ) );
    shape.scale( { mSandbox.getMetersPerPoint() / scalar, mSandbox.getMetersPerPoint() / scalar } );
    auto mesh = Triangulator( shape, 1 ).calcMesh( Triangulator::WINDING_ODD );
    mSandbox.createShape( mSandbox.toPhysics( loc ), mesh, scalar );
    loc.x += shape.calcBoundingBox().getWidth() * mSandbox.getPointsPerMeter() * scalar + 10.0f;
  }
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
