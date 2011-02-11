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
			
			const static float mScaling = 50.0f;	//50 pixels per meter sounds reasonable
			
			static Vec2f toScreen( b2Vec2 fin )
			{
				return Vec2f(fin.x, fin.y) * mScaling;
			}
			
			static b2Vec2 toPhysics( Vec2f fin )
			{
				return b2Vec2( fin.x/mScaling, fin.y/mScaling );
			}
			
			static float toPhysics( float fin )
			{
				return fin / mScaling;
			}
			
			static float toScreen( float fin )
			{
				return fin * mScaling;
			}
			
			static float radiansToDegrees( float rad )
			{
				return rad * 180.0f/M_PI;
			}
		};
		
	}

}