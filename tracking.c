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

gu_cartesian_coordinate calculate_difference(double forward, double left, double turn, double originalHeading)
{
    const double halfPi = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(90.0)));
    const millimetres_t x = d_to_mm_t(forward * cos(turn + originalHeading) + left * cos(turn + halfPi + originalHeading));
    const millimetres_t y = d_to_mm_t(forward * sin(turn + originalHeading) + left * sin(turn + halfPi + originalHeading));
    const gu_cartesian_coordinate differentialCoordinate = {x, y};
    return differentialCoordinate;
}

static gu_cartesian_coordinate check_counter_and_calculate_difference(gu_odometry_reading currentReading, gu_odometry_status currentStatus)
{
    if (currentReading.resetCounter != currentStatus.last_reading.resetCounter) {
        return calculate_difference(mm_t_to_d(currentReading.forward), mm_t_to_d(currentReading.left), rad_d_to_d(currentReading.turn), deg_d_to_rad_d(rad_d_to_d(currentStatus.my_position.heading)));
    }
    const gu_odometry_reading lastReading = currentStatus.last_reading;
    return calculate_difference(
        mm_t_to_d(currentReading.forward - lastReading.forward),
        mm_t_to_d(currentReading.left - lastReading.left),
        rad_d_to_d(currentReading.turn - lastReading.turn),
        deg_d_to_rad_d(rad_d_to_d(currentStatus.my_position.heading))
    );
}

static radians_d get_incremental_angle(gu_odometry_reading currentReading, gu_odometry_reading lastReading)
{
    if (currentReading.resetCounter != lastReading.resetCounter) {
        return currentReading.turn;
    }
    return currentReading.turn - lastReading.turn;
}

gu_odometry_status track(const gu_odometry_reading currentReading, const gu_odometry_status currentStatus)
{
    const gu_field_coordinate originalPosition = currentStatus.my_position;
    const gu_cartesian_coordinate differentialCoordinate = check_counter_and_calculate_difference(currentReading, currentStatus);
    gu_relative_coordinate differentialRelative = cartesian_coord_to_rr_coord(differentialCoordinate);
    const radians_d incrementalAngle = get_incremental_angle(currentReading, currentStatus.last_reading);
    const degrees_t newHeading = originalPosition.heading + rad_d_to_deg_t(incrementalAngle);
    differentialRelative.direction -= deg_t_to_deg_d(originalPosition.heading);
    const gu_field_coordinate newCoordinate = rr_coord_to_field_coord_from_source(differentialRelative, originalPosition, newHeading);
    const gu_cartesian_coordinate targetLocation = rr_coord_to_cartesian_coord_from_field(currentStatus.target, originalPosition);
    const gu_relative_coordinate newTarget = field_coord_to_rr_coord_to_target(newCoordinate, targetLocation);
    const gu_odometry_status newStatus = {newCoordinate, newTarget, currentReading};
    return newStatus;
}

gu_odometry_status create_status(const gu_odometry_reading initialReading, const gu_relative_coordinate object)
{
    const gu_field_coordinate originalPosition = {{0, 0}, 0};
    const gu_odometry_status status = {originalPosition, object, initialReading};
    return status;
}

gu_odometry_status create_status_for_self(const gu_odometry_reading initialReading)
{
    const gu_relative_coordinate self = {0.0, 0};
    return create_status(initialReading, self);
}

