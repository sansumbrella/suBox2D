/*
 * Copyright (c) 2010—2013, David Wicks, sansumbrella.com
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
/**
 Type declarations and header includes shared by files in library.
*/
#include <Box2D/Box2D.h>

namespace box2d
{
  typedef std::unique_ptr<b2Body, std::function<void(b2Body*)>>   unique_body_ptr;
  typedef std::unique_ptr<b2Joint, std::function<void(b2Joint*)>> unique_joint_ptr;
  typedef std::shared_ptr<b2Body> BodyRef;
  typedef std::weak_ptr<b2Body>   WeakBodyRef;

  // wrapper around a b2Shape to handle polymorphism more easily
  // does not own the shape due to box2d internals
  class Shape
  {
  public:
    explicit Shape( b2Shape *shape ):
    mShape( shape )
    {}
    b2Vec2 getCenter() const
    {
      switch ( mShape->GetType() )
      {
        case b2Shape::e_circle:
          return static_cast<b2CircleShape*>( mShape )->m_p;
          break;
        case b2Shape::e_polygon:
          return static_cast<b2PolygonShape*>( mShape )->m_centroid;
          break;
        default:
          break;
      }
      return b2Vec2( 0, 0 );
    }
    void offsetCenter( const ci::Vec2f &amount )
    {
      switch ( mShape->GetType() )
      {
        case b2Shape::e_circle:
          static_cast<b2CircleShape*>( mShape )->m_p.x += amount.x;
          static_cast<b2CircleShape*>( mShape )->m_p.y += amount.y;
          break;
        case b2Shape::e_polygon:
          static_cast<b2PolygonShape*>( mShape )->m_centroid.x += amount.x;
          static_cast<b2PolygonShape*>( mShape )->m_centroid.y += amount.y;
          break;
        default:
          break;
      }
    }
    b2Shape *get() { return mShape; }
  private:
    b2Shape  *mShape;
  };
}
