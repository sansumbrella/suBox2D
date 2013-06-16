/*
 *  BoundaryElement.h
 *  Joint2D
 *
 *  Created by David Wicks on 6/10/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 */

#pragma once

#include "PhysicsElement.h"
#include "Conversions.h"
#include "cinder/Rect.h"
#include "cinder/Vector.h"
#include "cinder/app/App.h"

namespace cinder {
	namespace box2d {
		class BoundaryElement : public PhysicsElement
		{
		public:
			BoundaryElement();
			BoundaryElement( Rectf screenBounds, float thickness=1.0f );
			void set( Rectf screenBounds, float thickness=1.0f );
			void draw();

		private:
			float mBoundThickness;
			Rectf mScreenBounds;
			b2PolygonShape floorShape, leftShape, rightShape, topShape;
		};
	}
}

