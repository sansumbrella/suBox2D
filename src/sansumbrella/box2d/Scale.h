/*
 * Copyright (c) 2013 David Wicks
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

/**
 Conversion methods for moving between screen and physics space.

 When setting the world scale, keep in mind that Box2D is designed to simulate
 objects between 10cm and 10m in size. With the Scale's default scale, that
 translates to screen sizes between 10 and 1000 points.
*/
namespace box2d
{
  class Scale
  {
  public:
    Scale();
    ~Scale();

    //! Set the number of screen points contained in a world meter
    //! default value is 100 (1 pixel == 1 centimeter)
    void setPointsPerMeter( float points );

    //! How many meters per point? Use this as scale factor from physics
    float getPointsPerMeter() const { return mPointsPerMeter; }

    //! Set the number of meters represented per screen point
    //! Default value is 0.01
    void setMetersPerPoint( float meters );
    float getMetersPerPoint() const { return mMetersPerPoint; }

    //! Convert from screen units to physical measurements
    template<typename T>
    inline T toPhysics( const T &points ) const { return points * mMetersPerPoint; }
    inline float toPhysics( float points ) const { return points * mMetersPerPoint; }

    //! Convert from physical measurements to screen units
    template<typename T>
    inline T fromPhysics( const T &physical_measure ) const { return physical_measure * mPointsPerMeter; }
    inline float fromPhysics( float physical_measure ) const { return physical_measure * mPointsPerMeter; }

  private:
    float   mPointsPerMeter = 100.0f;
    float   mMetersPerPoint = 1.0f / mPointsPerMeter;
  };

} // namespace box2d
