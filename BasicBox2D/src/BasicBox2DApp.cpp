#include "cinder/app/AppBasic.h"
#include <list>
#include "cinder/params/Params.h"
#include "cinder/Font.h"
#include "cinder/Utilities.h"
#include "cinder/Rand.h"

#include <b2cinder/b2cinder.h>

using namespace ci;
using namespace ci::app;
using namespace ci::box2d;
using namespace std;


class BasicBox2DApp : public AppBasic {
 public:
	void mouseDrag( MouseEvent event );
	void mouseDown( MouseEvent event );
	void update();
	void prepareSettings(Settings* settings);
	void setup();
	void draw();
	
private:	
	void addBox();
	
	Sandbox mSandbox;
	Font mFont;
};

void BasicBox2DApp::prepareSettings(Settings* settings)
{
	settings->setWindowSize(1024, 768);
	settings->setFrameRate(60.0f);
}

void BasicBox2DApp::setup()
{
	mSandbox.init();
	mSandbox.enableMouseInteraction(this);
	
	mFont = Font( "Hoefler Text", 12.0f );
}

void BasicBox2DApp::mouseDrag( MouseEvent event )
{
	if ( event.isAltDown() )
	{
		addBox();
	}
//	mSandbox.addBox( getMousePos(), Vec2f( Rand::randFloat(10.0f,40.0f), Rand::randFloat(10.0f,40.0f) ) );
}

void BasicBox2DApp::mouseDown( MouseEvent event )
{
	if( event.isAltDown() ) 
	{
		addBox();
	}

}

void BasicBox2DApp::update()
{
	mSandbox.update();
}

void BasicBox2DApp::addBox()
{
	BoxElement* b = new BoxElement( getMousePos(), Vec2f( Rand::randFloat(10.0f,40.0f), Rand::randFloat(10.0f,40.0f) ) );
	b->setColor( Color( CM_HSV, Rand::randFloat(0.0f,0.19f), 0.9f, 1.0f  ) );
	mSandbox.addElement(b);
}

void BasicBox2DApp::draw()
{
	gl::clear( Color::white() );
	gl::enableAlphaBlending();
	
	//draw all physics elements
	mSandbox.draw();
	//only draw the contact points in debug
	mSandbox.debugDraw(false, true);
	
	gl::drawString( "Framerate: " + toString(getAverageFps()), Vec2f( 10.0f, 10.0f ), Color::black(), mFont );
	gl::drawString( "Num bodies: " + toString(mSandbox.getBodyCount() ), Vec2f( 10.0f, 22.0f ), Color::black(), mFont );
	gl::drawString( "Num contacts: " + toString(mSandbox.getContactCount() ), Vec2f( 10.0f, 34.0f ), Color::black(), mFont );
	
//	params::InterfaceGl::draw();
}

// This line tells Cinder to actually create the application
CINDER_APP_BASIC( BasicBox2DApp, RendererGl )
