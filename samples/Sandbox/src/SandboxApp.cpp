#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "sansumbrella/b2cinder.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace ci::box2d;

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
  BoxElement* b = new BoxElement( loc, Vec2f( Rand::randFloat(10.0f,40.0f), Rand::randFloat(10.0f,40.0f) ) );
	b->setColor( Color( CM_HSV, Rand::randFloat(0.0f,0.19f), 0.9f, 1.0f  ) );
	mSandbox.addElement(b);
}

void SandboxApp::update()
{
  mSandbox.update();
}

void SandboxApp::draw()
{
	gl::clear( Color::white() );
  gl::enableAlphaBlending();
  mSandbox.draw();
  mSandbox.debugDraw( false, true );

  gl::drawString( "Framerate: " + to_string(getAverageFps()), Vec2f( 10.0f, 10.0f ), Color::black(), mFont );
	gl::drawString( "Num bodies: " + to_string(mSandbox.getBodyCount() ), Vec2f( 10.0f, 22.0f ), Color::black(), mFont );
	gl::drawString( "Num contacts: " + to_string(mSandbox.getContactCount() ), Vec2f( 10.0f, 34.0f ), Color::black(), mFont );
}

CINDER_APP_NATIVE( SandboxApp, RendererGl )
