#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"

#include "sansumbrella/Sandbox.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_App : public AppNative {
public:
  void setup();
  void update();
  void draw();

private:
  su::Sandbox   mSandbox;
};

void _TBOX_PREFIX_App::setup()
{
  mSandbox.createBoundaryRect( mSandbox.toPhysics( Rectf{getWindowBounds()} ) );
  // create some shapes
  Vec2f loc{ getWindowWidth() * 0.49, getWindowHeight() * 0.25 };
  Vec2f size{ 75.0f, 25.0f };
  mSandbox.createBox( mSandbox.toPhysics(loc), mSandbox.toPhysics(size) ).release();
  loc = { getWindowWidth() * 0.5, getWindowHeight() * 0.5 };
  float radius = 50.0f;
  mSandbox.createCircle( mSandbox.toPhysics(loc), mSandbox.toPhysics(radius) ).release();
}

void _TBOX_PREFIX_App::update()
{
  mSandbox.step();
}

void _TBOX_PREFIX_App::draw()
{
  // clear out the window with black
  gl::clear( Color( 0, 0, 0 ) );
  mSandbox.debugDraw();
}

CINDER_APP_NATIVE( _TBOX_PREFIX_App, RendererGl )
