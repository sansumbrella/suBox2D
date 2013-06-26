#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"

#include "sansumbrella/Sandbox.h"
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
using namespace std;

// something like this is a reasonable approach to managing a body in your
// own structure
struct Bubble
{
  Bubble( su::unique_b2Body_ptr &&b, const ci::Vec2f &l, float r ):
  body( move(b) ),
  loc( l ),
  radius( r )
  { // bouncy
    body->GetFixtureList()->SetRestitution( 0.6f );
  }
  // we can't copy-construct because of unique_ptr, but we can move-construct
  // necessary to have this ctor for std::vector support
  Bubble( Bubble &&other ):
  body( move( other.body ) ),
  loc( other.loc ),
  radius( other.radius )
  {}
  su::unique_b2Body_ptr body;
  ci::Vec2f             loc;
  float                 radius;
  // passing in scale on update lets you change it dynamically
  // also, objects don't need to know about where scale is stored/calculated
  void update( float scale )
  {
    loc.x = body->GetPosition().x * scale;
    loc.y = body->GetPosition().y * scale;
  }
  void draw()
  {
    gl::color( Color::white() );
    gl::pushModelView();
    gl::translate( loc );
    gl::drawSolidCircle( Vec2f::zero(), radius );
    gl::rotate( body->GetAngle() * 180 / M_PI );
    gl::color( Color::black() );
    gl::drawLine( Vec2f{ -radius, 0 }, Vec2f{ radius, 0 } );
    gl::popModelView();
  }
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
  vector<Bubble>  mBubbles;
};

void DrawingApp::prepareSettings(Settings *settings)
{
  settings->setWindowSize( 1024, 768 );
  settings->enableMultiTouch();
}

void DrawingApp::setup()
{
  auto bounds = mSandbox.toPhysics( Rectf{getWindowBounds()} );
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
  auto bounds = mSandbox.toPhysics( Rectf{getWindowBounds()} );
  for( int i = 0; i < 29; ++i )
  {
    Vec2f loc{ Rand::randFloat( bounds.getX1(), bounds.getX2() ), Rand::randFloat( bounds.getY1(), bounds.getY2() ) };
    float radius = Rand::randFloat( 10.0f, 100.0f );
    mBubbles.emplace_back( Bubble{ mSandbox.createCircle( loc, mSandbox.toPhysics(radius) ), mSandbox.fromPhysics(loc), radius } );
  }
}

void DrawingApp::update()
{
  mSandbox.step();
  for( auto &bubble : mBubbles )
  {
    bubble.update( mSandbox.getPointsPerMeter() );
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
