/*
 * Copyright (c) 2013 David Wicks, sansumbrella.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "SimpleControl.h"
#include "Sandbox.h"

using namespace box2d;
using namespace cinder;

SimpleControl::SimpleControl()
{}

SimpleControl::~SimpleControl()
{
  disconnectUserSignals();
}

void SimpleControl::connectUserSignals(ci::app::WindowRef window, Sandbox &sandbox, float scale )
{
  mMouseJoint.reset();
  b2BodyDef bodyDef;
  mMouseBody = sandbox.createBody(bodyDef);
  mMouseConnections[0] = window->getSignalMouseDown().connect( [=, &sandbox]( app::MouseEvent &event ){ mouseDown( event, sandbox, scale ); } );
  mMouseConnections[2] = window->getSignalMouseDrag().connect( [=]( app::MouseEvent &event ){ mouseDrag( event, scale ); } );
  mMouseConnections[1] = window->getSignalMouseUp().connect( [=]( app::MouseEvent &event ){ mouseUp( event ); } );
}

void SimpleControl::disconnectUserSignals()
{
  for( auto &connect : mMouseConnections )
  {
    connect.disconnect();
  }
}

// QueryCallback taken from box2d testbed
class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void SimpleControl::mouseDown( app::MouseEvent &event, Sandbox &sandbox, float scale )
{
	if( !mMouseJoint )
	{
    // Make a small box around the click point.
    b2Vec2 p{ event.getPos().x * scale, event.getPos().y * scale };
    b2AABB aabb;
    b2Vec2 d;
    d.Set(0.001f, 0.001f);
    aabb.lowerBound = p - d;
    aabb.upperBound = p + d;

    // Query the world for overlapping shapes.
    QueryCallback callback(p);
    sandbox.getWorld().QueryAABB(&callback, aabb);

    if (callback.m_fixture)
    {
      b2Body* body = callback.m_fixture->GetBody();
      body->SetAwake(true);
      b2MouseJointDef md;
      md.bodyA = mMouseBody.get();
      md.bodyB = body;
      md.target = p;
      md.maxForce = 2000.0f * body->GetMass();
      mMouseJoint = sandbox.createJoint(md);
    }
	}
}

void SimpleControl::mouseDrag( app::MouseEvent &event, float scale )
{
	if(mMouseJoint){
		static_cast<b2MouseJoint*>(mMouseJoint.get())->SetTarget( b2Vec2{ event.getPos().x * scale, event.getPos().y * scale } );
	}
}

void SimpleControl::mouseUp( app::MouseEvent &event )
{
  mMouseJoint.reset();
}
