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
#include <array>

/**
 Wrapper to make setting up a b2MouseJoint easy.
*/

namespace box2d
{
  class Sandbox;
  class SimpleControl
  {
  public:
    SimpleControl();
    ~SimpleControl();
    //! Enable user interaction with \a Sandbox through a b2MouseJoint
    void connectUserSignals( ci::app::WindowRef window, Sandbox &sandbox, float meters_per_point );
    //! Disable user interaction, called in destructor
    void disconnectUserSignals();
  private:
    // our mouse, for simple interaction
    unique_joint_ptr  mMouseJoint;
    // an empty body, modeled after the earlier box2d ground_body
    unique_body_ptr   mMouseBody;
    std::array<ci::signals::Connection, 3> mMouseConnections;
    // handlers basic user interaction
    void mouseDown( ci::app::MouseEvent &event, Sandbox &sandbox, float scale );
    void mouseDrag( ci::app::MouseEvent &event, float scale );
    void mouseUp( ci::app::MouseEvent &event );
  };
}

