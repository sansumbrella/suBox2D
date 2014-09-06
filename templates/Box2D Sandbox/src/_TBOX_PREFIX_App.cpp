#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"

#include "suBox2D.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_App : public AppNative {
  public:
	void setup();
	void update();
	void draw();

  private:
	// Sandbox wraps the b2World
	b2::Sandbox         mSandbox;
	// Scale for mapping between physics and screen coordinates
	b2::Scale           mScale;
	// Smart pointers store the bodies and destroy them when out of scope
	b2::unique_body_ptr mBox;
	b2::unique_body_ptr mCircle;
};

void _TBOX_PREFIX_App::setup()
{
	// Add a boundary shape to the simulation.
	// This shape's lifetime is managed by the sandbox.
	// It will be destroyed if you create a new boundary rect.
	mSandbox.createBoundaryRect( mScale.toPhysics( Rectf{getWindowBounds()} ) );

	// Add a box to the simulation.
	vec2 loc( getWindowWidth() * 0.49f, getWindowHeight() * 0.25f );
	vec2 size( 75.0f, 25.0f );
	mBox = mSandbox.createBox( mScale.toPhysics(loc), mScale.toPhysics(size) );

	// Add a circle to the simulation.
	loc = vec2( getWindowWidth() * 0.5f, getWindowHeight() * 0.5f );
	float radius = 50.0f;
	mCircle = mSandbox.createCircle( mScale.toPhysics(loc), mScale.toPhysics(radius) );
}

void _TBOX_PREFIX_App::update()
{
	mSandbox.step();
}

void _TBOX_PREFIX_App::draw()
{
	gl::clear( Color::black() );
	mSandbox.debugDraw( mScale.getPointsPerMeter() );
}

CINDER_APP_NATIVE( _TBOX_PREFIX_App, RendererGl )
