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
#include "stdio.h"

gu_cartesian_coordinate calculate_difference(double forward, double left, double turn)
{
    const double halfPi = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(90.0)));
    const millimetres_t x = d_to_mm_t(forward * cos(turn) + left * cos(turn + halfPi));
    const millimetres_t y = d_to_mm_t(forward * sin(turn) + left * sin(turn + halfPi));
    const gu_cartesian_coordinate differentialCoordinate = {x, y};
    return differentialCoordinate;
}

gu_relative_coordinate calculate_difference_relative(double forward, double left, double turn)
{
    return cartesian_coord_to_rr_coord(calculate_difference(forward, left, turn));
}

static gu_cartesian_coordinate check_counter_and_calculate_difference(gu_odometry_reading currentReading, gu_odometry_status currentStatus)
{
    if (currentReading.resetCounter != currentStatus.last_reading.resetCounter) {
        return calculate_difference(mm_t_to_d(currentReading.forward), mm_t_to_d(currentReading.left), rad_d_to_d(currentReading.turn));
    }
    const gu_odometry_reading lastReading = currentStatus.last_reading;
    return calculate_difference(
        mm_t_to_d(currentReading.forward - lastReading.forward),
        mm_t_to_d(currentReading.left - lastReading.left),
        rad_d_to_d(currentReading.turn - lastReading.turn)
    );
}

static gu_odometry_status track(gu_odometry_reading currentReading, gu_odometry_status currentStatus, bool isSelf)
{
    const gu_cartesian_coordinate lastRelativeLocation = currentStatus.cartesian_coordinate;
    const gu_cartesian_coordinate differentialCoordinate = check_counter_and_calculate_difference(currentReading, currentStatus);
    const millimetres_t x2 = isSelf ? lastRelativeLocation.x + differentialCoordinate.x : lastRelativeLocation.x - differentialCoordinate.x;
    const millimetres_t y2 = isSelf ? lastRelativeLocation.y + differentialCoordinate.y : lastRelativeLocation.y - differentialCoordinate.y;
    const gu_cartesian_coordinate newCoordinate = {x2, y2};
    const gu_odometry_status newStatus = {newCoordinate, {}, currentReading};
    return newStatus;
}

static radians_d get_incremental_angle(gu_odometry_reading currentReading, gu_odometry_reading lastReading)
{
    if (currentReading.resetCounter != lastReading.resetCounter) {
        return currentReading.turn;
    }
    return currentReading.turn - lastReading.turn;
}

static gu_odometry_status track_relative(gu_odometry_reading currentReading, gu_odometry_status currentStatus, bool isSelf)
{
    const gu_cartesian_coordinate cartesianCoordinate = rr_coord_to_cartesian_coord(currentStatus.relative_coordinate);
    if (isSelf) {
        const gu_odometry_status newStatus = {cartesianCoordinate, {}, currentStatus.last_reading};
        const gu_odometry_status calculatedStatus = track(currentReading, newStatus, true);
        const gu_relative_coordinate calculatedRelCoord = cartesian_coord_to_rr_coord(calculatedStatus.cartesian_coordinate);
        const gu_odometry_status newRelStatus = {{}, calculatedRelCoord, calculatedStatus.last_reading};
        return newRelStatus;
    }
    const gu_odometry_status newStatus = {{0, 0}, {}, currentStatus.last_reading};
    const gu_odometry_status calculatedStatus = track(currentReading, newStatus, true);
    const degrees_t heading = rad_d_to_deg_t(get_incremental_angle(currentReading, currentStatus.last_reading));
    const gu_field_coordinate source = {calculatedStatus.cartesian_coordinate, heading};
    const gu_relative_coordinate result = field_coord_to_rr_coord_to_target(source, cartesianCoordinate);
    const gu_odometry_status status = {{}, result, calculatedStatus.last_reading};
    return status;
}


gu_odometry_status track_coordinate(const gu_odometry_reading currentReading, const gu_odometry_status currentStatus)
{
    return track(currentReading, currentStatus, false); 
}

gu_odometry_status track_relative_coordinate(const gu_odometry_reading currentReading, const gu_odometry_status currentStatus)
{
    return track_relative(currentReading, currentStatus, false); 
}

gu_odometry_status track_self(const gu_odometry_reading currentReading, const gu_odometry_status currentStatus)
{
   return track(currentReading, currentStatus, true); 
}

gu_odometry_status track_self_relative(const gu_odometry_reading currentReading, const gu_odometry_status currentStatus)
{
    return track_relative(currentReading, currentStatus, true); 
}

gu_odometry_status create_cartesian_status(const gu_odometry_reading initialReading, const gu_cartesian_coordinate object)
{
    const gu_odometry_status status = {object, {}, initialReading};
    return status;
} 

gu_odometry_status create_relative_status(const gu_odometry_reading initialReading, const gu_relative_coordinate object)
{
    const gu_odometry_status status = {{}, object, initialReading};
    return status;
}















