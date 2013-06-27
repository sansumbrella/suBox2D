#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"

#include "sansumbrella/Sandbox.h"
#include "sansumbrella/Box2DScale.h"
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// something like this is a reasonable approach to managing a body in your
// own structure
class Bubble
{
public:
  Bubble( su::unique_b2Body_ptr &&b, const ci::Vec2f &l, float r ):
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
    gl::drawSolidCircle( Vec2f::zero(), mRadius );
    gl::rotate( mBody->GetAngle() * 180 / M_PI ); // box2d stores angle in radians
    gl::color( Color::black() );
    gl::drawLine( Vec2f{ -mRadius, 0 }, Vec2f{ mRadius, 0 } );
    gl::popModelView();
  }
private:
  su::unique_b2Body_ptr mBody;
  ci::Vec2f             mLoc;
  float                 mRadius;
  ci::Color             mColor;
};

class DrawingApp : public AppNative {
public:
  void prepareSettings( Settings *settings );
  void setup();
  void update();
  void draw();
  void resetBubbles();

private:
  su::Sandbox   	mSandbox;
  su::Box2DScale  mScale;
  vector<Bubble>  mBubbles;
};

void DrawingApp::prepareSettings(Settings *settings)
{
  settings->setWindowSize( 1024, 768 );
  settings->enableMultiTouch();
}

void DrawingApp::setup()
{
  auto bounds = mScale.toPhysics( Rectf{getWindowBounds()} );
  mSandbox.createBoundaryRect( bounds );

  resetBubbles();

  getWindow()->getSignalTouchesEnded().connect( [this]( TouchEvent &event ) -> void
                                               {
                                                 resetBubbles();
                                               }
                                               );
}

void DrawingApp::resetBubbles()
{
  mBubbles.clear();
  auto bounds = mScale.toPhysics( Rectf{getWindowBounds()} );
  for( int i = 0; i < 29; ++i )
  {
    Vec2f loc{ Rand::randFloat( bounds.getX1(), bounds.getX2() ), Rand::randFloat( bounds.getY1(), bounds.getY2() ) };
    float radius = Rand::randFloat( 10.0f, 100.0f );
    mBubbles.emplace_back( Bubble{ mSandbox.createCircle( loc, mScale.toPhysics(radius) ), mScale.fromPhysics(loc), radius } );
  }
}

void DrawingApp::update()
{
  mSandbox.step();
  for( auto &bubble : mBubbles )
  {
    bubble.update( mScale.getPointsPerMeter() );
  }
}

void DrawingApp::draw()
{
  // clear out the window with black
  gl::clear( Color( 0, 0, 0 ) );
  for( auto &bubble : mBubbles )
  {
    bubble.draw();
  }
}

CINDER_APP_NATIVE( DrawingApp, RendererGl )
