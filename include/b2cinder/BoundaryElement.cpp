/*
 *  BoundaryElement.cpp
 *  Joint2D
 *
 *  Created by David Wicks on 6/10/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#include "BoundaryElement.h"

using namespace cinder::box2d;

BoundaryElement::BoundaryElement()
{
}

BoundaryElement::BoundaryElement( Rectf screenBounds, float thickness )
{
	set( screenBounds, thickness );
}

void BoundaryElement::set( Rectf screenBounds, float thickness )
{
	mScreenBounds = screenBounds;
	mBoundThickness = thickness;

	if( mScreenBounds.calcArea() != 0 )
	{
		// half width and half height
		float w = Conversions::toPhysics( mScreenBounds.getWidth()/2.0f );
		float h = Conversions::toPhysics( mScreenBounds.getHeight()/2.0f );
		// center x, y


		Vec2f upperLeft = mScreenBounds.getUpperLeft();
		float x = Conversions::toPhysics( upperLeft.x ) + w;
		float y = Conversions::toPhysics( upperLeft.y ) + h;

		w += mBoundThickness;
		h += mBoundThickness;


		mBodyDef.position.Set( x, y );
		mBodyDef.type = b2_staticBody;

		// Left
		leftShape.SetAsBox( mBoundThickness, h, b2Vec2( -w, 0 ), 0 );
		mFixtureDef.shape = &leftShape;
		mFixtureDefs.push_back(mFixtureDef);

		// Right
		rightShape.SetAsBox( mBoundThickness, h, b2Vec2( w, 0 ), 0 );
		mFixtureDef.shape = &rightShape;
		mFixtureDefs.push_back(mFixtureDef);

		// Top
		topShape.SetAsBox( w, mBoundThickness, b2Vec2( 0, -h ), 0 );
		mFixtureDef.shape = &topShape;
		mFixtureDefs.push_back(mFixtureDef);

		// Bottom
		floorShape.SetAsBox( w, mBoundThickness, b2Vec2( 0, h ), 0 );
		mFixtureDef.shape = &floorShape;
		mFixtureDefs.push_back(mFixtureDef);
	}
}

void BoundaryElement::draw()
{

}