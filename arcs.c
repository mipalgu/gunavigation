/*
 * arcs.c 
 * gunavigation 
 *
 * Created by Callum McColl on 20/12/2019.
 * Copyright © 2019 Callum McColl. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this
 *    software must display the following acknowledgement:
 *
 *        This product includes software developed by Callum McColl.
 *
 * 4. Neither the name of the author nor the names of contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or
 * modify it under the above terms or under the terms of the GNU
 * General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see http://www.gnu.org/licenses/
 * or write to the Free Software Foundation, Inc., 51 Franklin Street,
 * Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "arcs.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define FORWARD_MIN_ARC_SPEED 30.0f // mm/s
#define FORWARD_MAX_ARC_SPEED 80.0f // mm/s
#define TURN_MIN_ARC_SPEED 10.0f // degrees/s
#define TURN_MAX_ARC_SPEED 40.0f // degrees/s

#define ABS_FORWARD_ARC_SPEED(distance) (MAX(MIN((distance / 2.0f), FORWARD_MAX_ARC_SPEED), FORWARD_MIN_ARC_SPEED))
#define ABS_TURN_ARC_SPEED(theta) (MAX((MIN(TURN_MAX_ARC_SPEED, roundf(theta / 180.0f) * TURN_MAX_ARC_SPEED)), TURN_MIN_ARC_SPEED))
#define TURN_ARC_SPEED(theta) (theta < 0 ? -ABS_TURN_ARC_SPEED(fabsf(theta)) : ABS_TURN_ARC_SPEED(theta))


/*
 * The arc can be defined as the following graphic:

                segmentDistance
                   _     _
             _                 _
       _                            _
   _    alpha                            _
_-----------------------------------------_
\  beta           distance               /
  \                                    /
    \                                /
      \                            /
        \                        /
          \                    / radius
            \   theta        /
              \            /
                \        /
                  \    /
                    \/

 */
gu_arc arc_for_coordinate(gu_coordinate coordinate)
{
  const float alpha = DEG2RAD(fabsf((float) coordinate.direction));
  const float distance = (float) coordinate.distance;
  const float beta = DEG2RAD(90.0f) - alpha;
  const float theta = DEG2RAD(180.0f) - (2.0f * beta);
  const float radius = sqrtf((distance * distance) / (2.0f * (1.0f - cosf(theta))));
  const float segmentDistance = radius * theta;
  const float returnDistance = roundf(segmentDistance);
  const float maxForwardSpeed = ABS_FORWARD_ARC_SPEED(returnDistance);
  const float maxTurnSpeed = fabsf(TURN_ARC_SPEED((float) coordinate.direction));
  const float straightSteps = segmentDistance / (maxForwardSpeed / 10.0f);
  const float turnSteps = fabsf((float) coordinate.direction) / maxTurnSpeed;
  const gu_arc arc = {(uint16_t) returnDistance, turnSteps, straightSteps, (int8_t) maxTurnSpeed, (int16_t) maxForwardSpeed, f_to_rad_f(theta)};
  return arc;
}

gu_arcspeed arcspeed_to_coordinate_on_arc(gu_coordinate coordinate, gu_arc arc)
{
  const float turnSteps = isinf(arc.turnSteps) || isnan(arc.turnSteps) ? 0.0f : arc.turnSteps;
  const float straightSteps = isinf(arc.straightSteps) || isnan(arc.straightSteps) ? 0.0f : arc.straightSteps;
  if (turnSteps == 0.0f && straightSteps == 0.0f)
  {
      const gu_arcspeed speed = { 0.0f, 0.0f };
      return speed;
  }
  if (turnSteps == 0.0f)
  {
      const gu_arcspeed speed = { 0, arc.maxForwardSpeed };
      return speed;
  }
  if (straightSteps == 0.0f)
  {
      const gu_arcspeed speed = { arc.maxTurnSpeed, 0 };
      return speed;
  }
  const float ratio = straightSteps / turnSteps;
  const float inverseRatio = turnSteps / straightSteps;
  const float forwardSpeed = ratio >= 1.0f ? ((float) arc.maxForwardSpeed) : ratio * ((float) arc.maxForwardSpeed);
  float turnSpeed = ratio >= 1.0f ? inverseRatio * ((float) arc.maxTurnSpeed) : ((float) arc.maxTurnSpeed);
  const float modifiedTurnSpeed = turnSpeed + turnSpeed * 0.1f;
  turnSpeed = isnan(modifiedTurnSpeed) ? turnSpeed : modifiedTurnSpeed;
  const int16_t actualForwardSpeed = MIN(arc.maxForwardSpeed, (int16_t) (roundf(forwardSpeed)));
  int8_t actualTurnSpeed = MIN(arc.maxTurnSpeed, (int8_t) (roundf(turnSpeed)));
  actualTurnSpeed = coordinate.direction < 0 ? -actualTurnSpeed : actualTurnSpeed;
  const gu_arcspeed speed = { actualTurnSpeed, actualForwardSpeed };
  return speed;
}

gu_arcspeed arcspeed_to_coordinate(gu_coordinate coordinate)
{
    gu_arc arc = arc_for_coordinate(coordinate);
    return arcspeed_to_coordinate_on_arc(coordinate, arc);
}
