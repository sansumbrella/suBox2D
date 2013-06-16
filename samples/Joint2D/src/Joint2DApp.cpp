#include "cinder/app/AppBasic.h"
#include "b2cinder/b2cinder.h"

using namespace ci;
using namespace ci::app;
using namespace ci::box2d;

class Joint2DApp : public AppBasic {
public:
	void prepareSettings(Settings *settings);
	
	void setup();
	void update();
	void draw();
	void shutdown();
	
	void keyDown( KeyEvent event );
	void mouseDrag( MouseEvent event );
	void fileDrop( FileDropEvent event );
	
private:
	Sandbox mSandbox;
};

void Joint2DApp::prepareSettings(Settings *settings)
{
	settings->setWindowSize(1024, 768);
	settings->setTitle("Joint2D");
}

void Joint2DApp::setup()
{
	mSandbox.init();
	
	mSandbox.enableMouseInteraction(this);
	
	// TODO:
	// create simple methods for creating joints between PhysicsElement objects
	// 
	
	// Create a Distance Joint
	
	BoxElement* a = new BoxElement( getWindowSize()/2.0f, Vec2f(50.0f, 50.0f) );
	BoxElement* b = new BoxElement( getWindowSize()/2.0f + Vec2f(100.0f, 0.0f), Vec2f(20.0f, 20.0f) );
	
	mSandbox.addElement(a);
	mSandbox.addElement(b);
	
	b2DistanceJointDef jointDef;
	
	jointDef.Initialize( a->getBody(), b->getBody(), a->getBody()->GetWorldCenter(), b->getBody()->GetWorldCenter() );
	jointDef.collideConnected = false;
//	jointDef.frequencyHz = 4.0f;
//	jointDef.dampingRatio = 0.5f;
	
	mSandbox.getWorld()->CreateJoint( &jointDef );
	
	// Create a Revolute Joint
	a = new BoxElement( Vec2f( 100.0f, 100.0f ), Vec2f(50.0f, 50.0f) );
	b = new BoxElement( Vec2f( 150.0f, 100.0f ), Vec2f(50.0f, 50.0f) );
	
	a->setColor( Color(1.0f, 0.0f, 0.1f) );
	b->setColor( Color(0.0f, 0.0f, 0.2f) );
	
	mSandbox.addElement(a);
	mSandbox.addElement(b);
	
	b2RevoluteJointDef revJointDef;
	revJointDef.Initialize( a->getBody(), b->getBody(), a->getBody()->GetWorldCenter() ); 
	
	mSandbox.getWorld()->CreateJoint( &revJointDef );
	
	
	// Create a Prismatic Joint
	b2PrismaticJointDef prisJointDef;
	b2Vec2 worldAxis(1.0f, 0.0f);
	
	a = new BoxElement( Vec2f(800.0f, 100.0f), Vec2f( 80.0f, 20.0f ) );
	b = new BoxElement( Vec2f(800.0f, 100.0f), Vec2f( 100.0f, 20.0f ) );
	
	a->setColor( Color( CM_HSV, 0.5f, 1.0f, 1.0f ) );
	b->setColor( Color( CM_HSV, 0.55f, 1.0f, 1.0f ) );
	
	mSandbox.addElement(a);
	mSandbox.addElement(b);
	
	prisJointDef.Initialize( a->getBody(), b->getBody(), a->getBody()->GetWorldCenter(), worldAxis );
	prisJointDef.lowerTranslation = Conversions::toScreen( -25.0f );
	prisJointDef.upperTranslation = Conversions::toPhysics( 50.0f );
	prisJointDef.enableLimit = true;
//	prisJointDef.maxMotorForce = 1.0f;
//	prisJointDef.motorSpeed = 0.0f;
//	prisJointDef.enableMotor = true;
	
	mSandbox.getWorld()->CreateJoint( &prisJointDef );
	
	// Create a Pulley Joint
	
	a = new BoxElement( Vec2f(200.0f, 400.0f), Vec2f( 10.0f, 60.0f ) );
	b = new BoxElement( Vec2f(250.0f, 400.0f), Vec2f( 20.0f, 60.0f ) );
	
	mSandbox.addElement(a);
	mSandbox.addElement(b);
	
	b2Vec2 anchor1 = a->getBody()->GetWorldCenter() - Conversions::toPhysics( Vec2f(0, 20.0f) );
	b2Vec2 anchor2 = b->getBody()->GetWorldCenter() - Conversions::toPhysics( Vec2f(0, 20.0f) );
	b2Vec2 ground1 = Conversions::toPhysics( Vec2f( 200.0f, 0.0f ) );
	b2Vec2 ground2 = Conversions::toPhysics( Vec2f( 250.0f, 0.0f ) );
	
	float32 ratio = 1.0f;
	
	b2PulleyJointDef pulleyJointDef;
	pulleyJointDef.Initialize( a->getBody(), b->getBody(), ground1, ground2, anchor1, anchor2, ratio );
	pulleyJointDef.maxLengthA = Conversions::toPhysics( 400.0f );
	pulleyJointDef.maxLengthB = Conversions::toPhysics( 600.0f );
	
	mSandbox.getWorld()->CreateJoint(&pulleyJointDef);
	
	// Create a Gear Joint
	
	
}

void Joint2DApp::update()
{
	mSandbox.update();
}

void Joint2DApp::draw()
{
	gl::clear(Color::white());
	gl::enableAlphaBlending();
	
	mSandbox.draw();
//	mSandbox.debugDraw(true, true);
}

void Joint2DApp::mouseDrag( MouseEvent event )
{
	
}

void Joint2DApp::keyDown( KeyEvent event )
{
	switch( event.getChar() ){
		default:
		break;
	}
}

void Joint2DApp::fileDrop( FileDropEvent event )
{
	
}

void Joint2DApp::shutdown()
{
	
}


// This line tells Cinder to actually create the application
CINDER_APP_BASIC( Joint2DApp, RendererGl )
