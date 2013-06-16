#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"
#include "sansumbrella/b2cinder.h"
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
  void addBox( const ci::Vec2f &loc );
  void createCrazyShape();
  void applyForceToShape( const ci::Vec2f &force );
	Sandbox   mSandbox;
	Font      mFont;
  b2Body*   mCrazyBody = nullptr;
};

void SandboxApp::prepareSettings(Settings *settings)
{
}

void SandboxApp::setup()
{
  mSandbox.init();
  mSandbox.connectUserSignals( getWindow() );
  mFont = Font( "Hoefler Text", 12.0f );
  createCrazyShape();
}

void SandboxApp::keyDown(KeyEvent event)
{
  switch ( event.getCode() )
  {
    case KeyEvent::KEY_c:
      createCrazyShape();
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
}

void SandboxApp::applyForceToShape(const ci::Vec2f &force)
{
  mCrazyBody->ApplyForceToCenter( b2Vec2{ force.x, force.y } );
//  mCrazyBody->ApplyForce( b2Vec2{ force.x, force.y}, b2Vec2{ mSandbox.toPhysics(getWindowWidth()/2), mSandbox.toPhysics(getWindowHeight()) } );
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
  mSandbox.createCircle( loc, Rand::randFloat( 5.0f, 20.0f ) );
//  mSandbox.createBox( loc, Vec2f( Rand::randFloat(10.0f,40.0f), Rand::randFloat(10.0f,40.0f) ) );
}

void SandboxApp::createCrazyShape()
{
  double d1 = getElapsedSeconds();
  float d = Rand::randFloat( 0.5, 2.0f );
  Path2d outline;
  outline.moveTo( cos( 0 ) * d, sin( 0 ) * d );
  const int points = 12;
  for( int i = 1; i < points; ++i )
  {
    float t = lmap<float>( i, 0, points, 0, M_PI * 2 );
    d = Rand::randFloat( 0.5, 2.0f );
    outline.lineTo( cos( t ) * d, sin( t ) * d  );
  }

  vector<b2Vec2> hull_vertices( outline.getNumPoints() );
  for( int i = 0; i < hull_vertices.size(); ++i )
  {
    hull_vertices[i] = b2Vec2{ outline.getPoint(i).x, outline.getPoint(i).y };
  }
  if( mCrazyBody ){ mSandbox.destroyBody( mCrazyBody ); }
  mCrazyBody = mSandbox.createFanShape( mSandbox.toPhysics( getWindowSize() / 2 ), hull_vertices );
//  mCrazyBody = mSandbox.createShape( mSandbox.toPhysics( getWindowSize() / 2 ), Triangulator( outline, 1.0 ).calcMesh( Triangulator::WINDING_ODD ) );
  double d2 = getElapsedSeconds();
  cout << "Creating shape required: " << (d2 - d1) * 1000 << "ms" << endl;
}

void SandboxApp::update()
{
  mSandbox.step();
}

void SandboxApp::draw()
{
	gl::clear( Color::black() );

  mSandbox.debugDraw();

  gl::enableAlphaBlending();
  gl::drawString( "Framerate: " + to_string(getAverageFps()), Vec2f( 10.0f, 10.0f ), Color::white(), mFont );
	gl::drawString( "Num bodies: " + to_string(mSandbox.getBodyCount() ), Vec2f( 10.0f, 22.0f ), Color::white(), mFont );
	gl::drawString( "Num contacts: " + to_string(mSandbox.getContactCount() ), Vec2f( 10.0f, 34.0f ), Color::white(), mFont );
}

CINDER_APP_NATIVE( SandboxApp, RendererGl )
