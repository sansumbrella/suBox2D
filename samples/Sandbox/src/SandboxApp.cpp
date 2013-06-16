#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"
#include "sansumbrella/b2cinder.h"

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
	void update();
	void draw();
private:
  void addBox( const ci::Vec2f &loc );
	Sandbox   mSandbox;
	Font      mFont;
};

void SandboxApp::prepareSettings(Settings *settings)
{
}

void SandboxApp::setup()
{
  mSandbox.init();
  mSandbox.connectUserSignals( getWindow() );
  mFont = Font( "Hoefler Text", 12.0f );
}

void SandboxApp::mouseDown( MouseEvent event )
{
  if( !event.isAltDown() )
	{
		addBox( event.getPos() );
	}
}

void SandboxApp::mouseDrag(MouseEvent event)
{
  if( !event.isAltDown() )
	{
		addBox( event.getPos() );
	}
}

void SandboxApp::addBox( const ci::Vec2f &loc )
{
  mSandbox.createBox( loc, Vec2f( Rand::randFloat(10.0f,40.0f), Rand::randFloat(10.0f,40.0f) ) );
}

void SandboxApp::update()
{
  mSandbox.step();
}

void SandboxApp::draw()
{
	gl::clear( Color::white() );
  gl::enableAlphaBlending();
//  mSandbox.draw();
  mSandbox.debugDraw( true, true );

  gl::drawString( "Framerate: " + to_string(getAverageFps()), Vec2f( 10.0f, 10.0f ), Color::black(), mFont );
	gl::drawString( "Num bodies: " + to_string(mSandbox.getBodyCount() ), Vec2f( 10.0f, 22.0f ), Color::black(), mFont );
	gl::drawString( "Num contacts: " + to_string(mSandbox.getContactCount() ), Vec2f( 10.0f, 34.0f ), Color::black(), mFont );
}

CINDER_APP_NATIVE( SandboxApp, RendererGl )
