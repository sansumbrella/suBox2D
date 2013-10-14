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

#include "ContactListener.h"

using namespace box2d;

ContactListener::~ContactListener()
{
  if( mWorld )
  {
    mWorld->SetContactListener( nullptr );
  }
}

ContactListener::ContactListener( ContactListener &other )
{
  listenToWorld( other.mWorld );
  other.mWorld = nullptr;
}

ContactListener ContactListener::operator=( ContactListener &rhs )
{
  ContactListener ret( rhs );
  return ret;
}

void ContactListener::listenToWorld( b2World *world )
{
  if( world )
  { // unregister previous world
    world->SetContactListener( nullptr );
  }
  mWorld = world;
  mWorld->SetContactListener( this );
}

void ContactListener::BeginContact( b2Contact *contact )
{
	mBeginContact( contact );
}

void ContactListener::EndContact( b2Contact *contact )
{
	mEndContact( contact );
}

void ContactListener::PreSolve( b2Contact *contact, const b2Manifold *oldManifold )
{
	mPreSolve( contact, oldManifold );
}

void ContactListener::PostSolve( b2Contact *contact, const b2ContactImpulse *impulse )
{
	mPostSolve( contact, impulse );
}
