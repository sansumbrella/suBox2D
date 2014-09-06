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
#include "cinder/Triangulate.h"
#include "cinder/Rand.h"
#include "cinder/Font.h"

#include "suBox2D.h"

using namespace ci;
using namespace ci::app;
using namespace std;

/**
 Example demonstrating general use of the Sandbox, creating a variety of bodies.
 Uses the debugDraw() method to display all physics bodies on screen.
 Uses SimpleControl to enable simple mouse/touch interaction.

 Mouse to interact.
 Hold alt/option when dragging to add circles to the world.
 'c' to create a new spiky body
 'r' to reset (destroy all bodies)
 */

class ShapeCreationApp : public AppNative {
public:
  void prepareSettings( Settings *settings );
  void setup();
  void mouseDown( MouseEvent event );
  void mouseDrag( MouseEvent event );
  void keyDown( KeyEvent event );
  void update();
  void draw();
  void reset();
  void createBodies();
  void createTextShape( const std::string &text, const ci::vec2 &top_left );
  void createSpikyShape();
private:
  b2::Sandbox                   mSandbox;
  b2::Scale                     mScale;
  b2::SimpleControl             mControl;
  b2::unique_body_ptr           mSpikyBody;
  vector<b2::unique_body_ptr>   mBodies;
};

void ShapeCreationApp::prepareSettings( Settings *settings )
{
  settings->setWindowSize( 1024, 768 );
}

void ShapeCreationApp::setup()
{
  mSandbox.createBoundaryRect( mScale.toPhysics( Rectf{getWindowBounds()} ) );
  mControl.connectUserSignals( getWindow(), mSandbox, mScale.getMetersPerPoint() );
  createBodies();
}

void ShapeCreationApp::reset()
{
  mBodies.clear();
  createBodies();
}

void ShapeCreationApp::createBodies()
{
  createTextShape( "Collapsed", vec2{ 200.0f, 100.0f } );
  createSpikyShape();

  vec2 loc = mScale.toPhysics( vec2{ Rand::randFloat(getWindowWidth()), getWindowHeight() / 2 + Rand::randFloat(getWindowHeight()/2) } );
  float radius = mScale.toPhysics( Rand::randFloat( 20.0f, 80.0f ) );
  // Create a circle at loc with radius (and hold onto its ptr)
  mBodies.emplace_back( mSandbox.createCircle( loc, radius ) );

  loc = mScale.toPhysics( vec2{ Rand::randFloat(getWindowWidth()), getWindowHeight() / 2 + Rand::randFloat(getWindowHeight()/2) } );
  vec2 size = mScale.toPhysics( vec2{ Rand::randFloat( 20.0f, 100.0f ), Rand::randFloat( 20.0f, 100.0f ) } );
  // Create a box at loc with size (and hold onto its ptr)
  mBodies.emplace_back( mSandbox.createBox( loc, size ) );
}

void ShapeCreationApp::createSpikyShape()
{
  // create a ring of points with random radii
  vector<b2Vec2> hull_vertices( 12 );
  for( int i = 0; i < hull_vertices.size(); ++i )
  {
    float t = lmap<float>( i, 0, hull_vertices.size(), 0, M_PI * 2 );
    float r = Rand::randFloat( 0.1, 2.0f );
    float x = cos( t ) * r;
    float y = sin( t ) * r;
    hull_vertices[i] = b2Vec2{ x, y };
  }
  // create shape as a fan (works well with the radial vertices we defined)
  // much faster than generic triangulation, but only works on certain shapes
  // will crash if the triangles have negative (clockwise) area
  mSpikyBody = mSandbox.createFanShape( mScale.toPhysics( vec2(getWindowSize() / 2) ), hull_vertices );
}

void ShapeCreationApp::createTextShape( const string &text, const vec2 &top_left )
{
  // create shape using Cinder's Triangulator to calculate the triangles
  // works on almost any arbitrary shape
  Font f{ "Arial", 128.0f };
  vec2 loc{ top_left };
  // de-res the text before getting the physics outline
  // this scalar is just to make it much smaller when we triangulate
  float scalar = 10000.0f;
  for( auto &c : text )
  {
    auto shape = f.getGlyphShape( f.getGlyphChar( c ) );
    shape.scale( { mScale.toPhysics( 1 / scalar ), mScale.toPhysics( 1 / scalar ) } );
    auto mesh = Triangulator( shape, 1 ).calcMesh( Triangulator::WINDING_ODD );
    mBodies.emplace_back( mSandbox.createShape( mScale.toPhysics( loc ), mesh, scalar ) );
    loc.x += mScale.fromPhysics( shape.calcBoundingBox().getWidth() * scalar ) + 10.0f;
  }
}

void ShapeCreationApp::keyDown( KeyEvent event )
{
  switch ( event.getChar() )
  {
    case 'c':
      createSpikyShape();
      break;
    case 'r':
      reset();
      break;
    default:
      break;
  }
}

void ShapeCreationApp::mouseDown( MouseEvent event )
{
  if( event.isAltDown() )
  {
    mBodies.emplace_back( mSandbox.createCircle( mScale.toPhysics( vec2{event.getPos()} ), Rand::randFloat( 0.1f, 0.2f ) ) );
  }
}

void ShapeCreationApp::mouseDrag( MouseEvent event )
{
  if( event.isAltDown() )
  {
    mBodies.emplace_back( mSandbox.createCircle( mScale.toPhysics( vec2{event.getPos()} ), Rand::randFloat( 0.1, 0.2f ) ) );
  }
}

void ShapeCreationApp::update()
{
  mSandbox.step();
}

void ShapeCreationApp::draw()
{
  // clear out the window with black
  gl::clear( Color( 0, 0, 0 ) );
  mSandbox.debugDraw( mScale.getPointsPerMeter() );
}

CINDER_APP_NATIVE( ShapeCreationApp, RendererGl( RendererGl::Options().antiAliasing( RendererGl::AA_MSAA_4 ) ) )
