/*
 * tracking.c 
 * gunavigation 
 *
 * Created by Morgan McColl on 12/10/2020.
 * Copyright © 2020 Morgan McColl. All rights reserved.
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
 *        This product includes software developed by Morgan McColl.
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

#include "tracking.h"
#include "math.h"

gu_cartesian_coordinate calculate_difference(double forward, double left, double turn)
{
    const double halfPi = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(90.0)));
    const centimetres_t x = mm_t_to_cm_t(d_to_mm_t(forward * cos(turn) + left * cos(turn + halfPi)));
    const centimetres_t y = mm_t_to_cm_t(d_to_mm_t(forward * sin(turn) + left * sin(turn + halfPi)));
    const gu_cartesian_coordinate differentialCoordinate = {x, y};
    return differentialCoordinate;
}

gu_relative_coordinate calculate_difference_relative(double forward, double left, double turn)
{
    return cartesian_coord_to_rr_coord(calculate_difference(forward, left, turn));
}

static gu_odometry_status track_with_reset(gu_odometry_reading lastReading, gu_odometry_reading currentReading, gu_odometry_status currentStatus, bool isSelf)
{
    const gu_cartesian_coordinate lastRelativeLocation = currentStatus.cartesian_coordinate;
    double currentTurn = rad_d_to_d(currentReading.turn);
    const double currentForward = mm_t_to_d(currentReading.forward);
    const double currentLeft = mm_t_to_d(currentReading.left);
    const gu_cartesian_coordinate differentialCoordinate = calculate_difference(currentForward, currentLeft, rad_d_to_d(currentStatus.turn + lastReading.turn) + currentTurn);
    const millimetres_t newForward = currentReading.forward;
    const millimetres_t newLeft = currentReading.left;
    const radians_d newTurn = currentStatus.turn + lastReading.turn;
    const centimetres_t x2 = isSelf ? lastRelativeLocation.x + differentialCoordinate.x : lastRelativeLocation.x - differentialCoordinate.x;
    const centimetres_t y2 = isSelf ? lastRelativeLocation.y + differentialCoordinate.y : lastRelativeLocation.y - differentialCoordinate.y;
    gu_cartesian_coordinate newCoordinate = {x2, y2};
    gu_odometry_status newStatus = {newForward, newLeft, newTurn, newCoordinate, {}};
    return newStatus;
}

static gu_odometry_status track_without_reset(gu_odometry_reading currentReading, gu_odometry_status currentStatus, bool isSelf)
{
    const gu_cartesian_coordinate lastRelativeLocation = currentStatus.cartesian_coordinate;
    double currentTurn = rad_d_to_d(currentReading.turn);
    const double currentForward = mm_t_to_d(currentReading.forward);
    const double currentLeft = mm_t_to_d(currentReading.left);
    double dForward = currentForward - mm_t_to_d(currentStatus.forward);
    double dLeft = currentLeft - mm_t_to_d(currentStatus.left);
    const gu_cartesian_coordinate differentialCoordinate = calculate_difference(dForward, dLeft, rad_d_to_d(currentStatus.turn) + currentTurn);
    const millimetres_t newForward = d_to_mm_t(dForward) + currentStatus.forward;
    const millimetres_t newLeft = d_to_mm_t(dLeft) + currentStatus.left;
    const centimetres_t x2 = isSelf ? lastRelativeLocation.x + differentialCoordinate.x : lastRelativeLocation.x - differentialCoordinate.x;
    const centimetres_t y2 = isSelf ? lastRelativeLocation.y + differentialCoordinate.y : lastRelativeLocation.y - differentialCoordinate.y;
    gu_cartesian_coordinate newCoordinate = {x2, y2};
    gu_odometry_status newStatus = {newForward, newLeft, currentStatus.turn, newCoordinate, {}};
    return newStatus;
}

gu_odometry_status track_coordinate(
    const gu_odometry_reading lastReading,
    const gu_odometry_reading currentReading,
    const gu_odometry_status currentStatus
)
{
    if (currentReading.resetCounter != lastReading.resetCounter) {
        return track_with_reset(lastReading, currentReading, currentStatus, false);
    }
    return track_without_reset(currentReading, currentStatus, false);
}

gu_odometry_status track_relative_coordinate(
    const gu_odometry_reading lastReading,
    const gu_odometry_reading currentReading,
    const gu_odometry_status currentStatus
)
{
    gu_relative_coordinate relativeCoordinate = currentStatus.relative_coordinate;
    gu_cartesian_coordinate cartesianCoordinate = rr_coord_to_cartesian_coord(relativeCoordinate);
    const gu_odometry_status newStatus = {currentStatus.forward, currentStatus.left, currentStatus.turn, cartesianCoordinate, {}};
    gu_odometry_status calculatedStatus = track_coordinate(lastReading, currentReading, newStatus);
    gu_cartesian_coordinate calculatedCoordinate = calculatedStatus.cartesian_coordinate;
    gu_relative_coordinate calculatedRelCoord = cartesian_coord_to_rr_coord(calculatedCoordinate); 
    calculatedStatus.relative_coordinate = calculatedRelCoord;
    return calculatedStatus;
}

gu_odometry_status track_self(
    const gu_odometry_reading lastReading,
    const gu_odometry_reading currentReading,
    const gu_odometry_status currentStatus
)
{
   if (currentReading.resetCounter != lastReading.resetCounter) {
       return track_with_reset(lastReading, currentReading, currentStatus, true);
   }
   return track_without_reset(currentReading, currentStatus, true);
}

gu_odometry_status track_self_relative(
    const gu_odometry_reading lastReading,
    const gu_odometry_reading currentReading,
    const gu_odometry_status currentStatus
)
{
    gu_relative_coordinate relativeCoordinate = currentStatus.relative_coordinate;
    gu_cartesian_coordinate cartesianCoordinate = rr_coord_to_cartesian_coord(relativeCoordinate);
    const gu_odometry_status newStatus = {currentStatus.forward, currentStatus.left, currentStatus.turn, cartesianCoordinate, {}};
    gu_odometry_status calculatedStatus = track_self(lastReading, currentReading, newStatus);
    gu_cartesian_coordinate calculatedCoordinate = calculatedStatus.cartesian_coordinate;
    gu_relative_coordinate calculatedRelCoord = cartesian_coord_to_rr_coord(calculatedCoordinate); 
    calculatedStatus.relative_coordinate = calculatedRelCoord;
    return calculatedStatus;
}
















