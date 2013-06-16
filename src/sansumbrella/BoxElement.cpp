/*
 *  BoxElement.cpp
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/9/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#include "BoxElement.h"

using namespace cinder::box2d;

BoxElement::BoxElement( Vec2f pos, Vec2f size, bool isDynamic )
{
	mBounds = Rectf( -size.x/2.0f, -size.y/2.0f, size.x/2.0f, size.y/2.0f );

	mBodyDef.position.Set( Conversions::toPhysics( pos.x ),
                        Conversions::toPhysics( pos.y ) );

	mShape.SetAsBox( Conversions::toPhysics( size.x/2.0f ),
                  Conversions::toPhysics( size.y/2.0f ) );

	mFixtureDef.shape = &mShape;
	mFixtureDef.friction = 0.3f;
	mFixtureDef.restitution = 0.15f;

	if( isDynamic ){
		mBodyDef.type = b2_dynamicBody;
		mFixtureDef.density = 1.0f;
	}


	mFixtureDefs.push_back( mFixtureDef );

	mColor = Color( 1.0f, 1.0f, 0.0f );
}

void BoxElement::draw()
{
	Vec2f pos = Conversions::toScreen( mBody->GetPosition() );
	float t = Conversions::radiansToDegrees( mBody->GetAngle() );

	gl::color( mColor );

	glPushMatrix();
	gl::translate( pos );
	gl::rotate( t );

	gl::drawSolidRect( mBounds );
	
	glPopMatrix();
}