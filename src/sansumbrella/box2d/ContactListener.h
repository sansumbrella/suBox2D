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

#pragma once
#include "Common.h"

 namespace box2d
 {
  /**
  ContactListener:
  Facility for listening to Box2D contact events.
  Assign your own functions to be called when contact events occur.
  See the Box2D manual for more information on the event timing and use.
  There can be only one ContactListener per b2World, so we unregister instances
  in the copy constructor and on assignment. This leaves the newly assigned
  ContactListener as the sole receiver of events, which is generally what you
  would expect.
  */
  class ContactListener : public b2ContactListener
  {
  public:
    ContactListener() = default;
    ~ContactListener();
    ContactListener( ContactListener &other );
    ContactListener operator= (ContactListener &rhs);
    typedef std::function<void (b2Contact*)> ContactFn;
    typedef std::function<void (b2Contact*, const b2Manifold*)> PreSolveFn;
    typedef std::function<void (b2Contact*, const b2ContactImpulse*)> PostSolveFn;
    void listenToWorld( b2World *world );
    //! assign function to be called when contacts start
    void setBeginContactFn( const ContactFn &fn ){ mBeginContact = fn; }
    //! assign function to be called when contacts end
    void setEndContactFn( const ContactFn &fn ){ mEndContact = fn; }
    //! assign function to be called before collision resolution (e.g. platforms)
    void setPreSolveFn( const PreSolveFn &fn ){ mPreSolve = fn; }
    //! assign function to be called after collision resolution (e.g. breaking shapes)
    void setPostSolveFn( const PostSolveFn &fn ){ mPostSolve = fn; }
    // virtual methods called by box2d
    void BeginContact( b2Contact *contact ) override;
    void EndContact( b2Contact *contact ) override;
    void PreSolve( b2Contact *contact, const b2Manifold *oldManifold ) override;
    void PostSolve( b2Contact *contact, const b2ContactImpulse *impulse ) override;
    // empty default methods so we don't need conditional in methods
    // set fn's to one of these to "unset" your own handler
    static void noopContact( b2Contact* ){}
    static void noopPreSolve( b2Contact*, const b2Manifold* ){}
    static void noopPostSolve( b2Contact*, const b2ContactImpulse* ){}
  private:
    ContactFn   mBeginContact = &noopContact;
    ContactFn   mEndContact= &noopContact;
    PreSolveFn  mPreSolve = &noopPreSolve;
    PostSolveFn mPostSolve = &noopPostSolve;
    b2World     *mWorld = nullptr; // the world we're listening to
  };
}
