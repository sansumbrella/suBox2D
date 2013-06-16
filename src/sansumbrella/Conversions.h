/*
 *  Utils.h
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/7/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#pragma once
#include "cinder/Vector.h"
#include "Box2D/Box2D.h"

namespace cinder{

	namespace box2d {
		
		struct Conversions {
			
            static float getScaling(){ static float scaling = 50.0f; return scaling; }
			
			static Vec2f toScreen( b2Vec2 fin )
			{
				return Vec2f(fin.x, fin.y) * getScaling();
			}
			
			static b2Vec2 toPhysics( Vec2f fin )
			{
				return b2Vec2( fin.x/getScaling(), fin.y/getScaling() );
			}
			
			static float toPhysics( float fin )
			{
				return fin / getScaling();
			}
			
			static float toScreen( float fin )
			{
				return fin * getScaling();
			}
			
			static float radiansToDegrees( float rad )
			{
				return rad * 180.0f/M_PI;
			}
		};
		
	}

}