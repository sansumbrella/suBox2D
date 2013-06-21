/*
 *  DynamicBody.h
 *  BasicBox2D
 *
 *  Created by David Wicks on 6/9/10.
 *  Copyright 2010 David Wicks. All rights reserved.
 *
 *	Base class for elements added to Box2D simulation
 *	Sandbox casts b2Body::UserData to this type for rendering
 *
 *
 */

#pragma once

/*
	TODO:
	Create a class that is a good default for use as b2Body::UserData
*/
/*
#include <Box2D/Box2D.h>
#include <list>
#include "cinder/app/App.h"

namespace cinder {
	namespace box2d {
		class PhysicsElement {

		public:
			PhysicsElement();
			virtual ~PhysicsElement(){}
			// things necessary for defining objects in physics simulation
			b2BodyDef* getBodyDef(){ return &mBodyDef; }

			// all fixtures on our body (usually one)
			// might make this an array and have a numFixtures member...
			std::list<b2FixtureDef>* getFixtureDefs(){ return &mFixtureDefs; }

			// careful with this one; mBody is undefined until set
			b2Body* getBody(){ return mBody; }
			// called when the Sandbox destroys our physics body
			// deletes this by default since we generally don't keep a reference
			// to objects outside of the b2body
			// *might* move to reference-counted userdata
			virtual void destroyBody(){}
			// creates a circular reference with the b2Body in the simulation
			void setBody( b2Body* b ){ mBody = b; b->SetUserData(this); }

			void applyForce( Vec2f force, Vec2f pos ){ mBody->ApplyForce( Conversions::toPhysics(force), Conversions::toPhysics(pos) ); }

			// update any settings applied to the body
			virtual void update(){}
			// render the body on screen
			virtual void draw(){}
			// should the object be removed from simulation?
			virtual bool isDead(){ return false; }

			//
			Vec2f getScreenPos(){ return Conversions::toScreen( mBody->GetPosition() ); }
			float getAngleDegrees(){ return Conversions::radiansToDegrees( mBody->GetAngle() ); }

			// add special attributes

		private:


		protected:
			b2BodyDef mBodyDef;

			// temporary fixture def and list for all defs
			b2FixtureDef mFixtureDef;
			std::list<b2FixtureDef> mFixtureDefs;
			// only gets set when the body is added to the b2World
			b2Body* mBody;

		};
	}
}

*/