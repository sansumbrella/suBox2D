/*
 *  BoxElement.h
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/9/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 *  Not sure about the naming of these files/classes right now
 *
 */

#pragma once
#include "b2cinder/Conversions.h"
#include "b2cinder/PhysicsElement.h"
#include "cinder/Vector.h"
#include "cinder/Color.h"
#include "cinder/app/AppBasic.h"

namespace cinder{
	namespace box2d {
		
		// wraps the creation of box-shaped elements
		// sets reasonable defaults and allows easy tweaking of parameters
		
		class BoxElement : public PhysicsElement {
		public:
			// Create a BoxElement, isDynamic flag is currently ignored (thinking about it)
			BoxElement(){}
			~BoxElement(){}
			BoxElement( Vec2f pos, Vec2f size, bool isDynamic=true );
			// draws a box
			virtual void draw();
			void destroyBody(){ delete this; }
			void setColor( Color c ){ mColor = c; }
			
		private:
			Color mColor;
			Rectf mBounds;
			b2PolygonShape mShape;
			
		};
		
	}

}