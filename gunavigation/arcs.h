/*
 * arcs.h 
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

#ifndef GUNAVIGATION_ARCS_H
#define GUNAVIGATION_ARCS_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <guunits/Coordinate.h>

/**
 * Return a number of parameters which are calculated
 * from a coordinate.
 *
 */
typedef struct gu_arc
{
    /**
     * The length of the arc.
     */
    uint16_t segmentLength;

    /**
     * The number of steps it takes to turn along the arc. 
     */
    float turnSteps;

    /**
     * The number of steps it takes to walk straight along the arc.
     */
    float straightSteps;

    /**
     * The maxium turning speed.
     *
     * This is in units per second.
     */
    int8_t maxTurnSpeed;

    /**
     * The maximum forward speed.
     *
     * This is in units per second.
     */
    int16_t maxForwardSpeed;

    /**
     * The angle between the two radius lines.
     */
    radians_f theta;

} gu_arc;

typedef struct gu_arcspeed {

    int8_t turnSpeed;

    int16_t forwardSpeed;

} gu_arcspeed;


/**
 * Calculate the arc_parameters to a given coordinate.
 */
gu_arc arc_for_coordinate(gu_coordinate coordinate);

gu_arcspeed arcspeed_to_coordinate_on_arc(gu_coordinate coordinate, gu_arc arc);

gu_arcspeed arcspeed_to_coordinate(gu_coordinate coordinate);

#ifdef __cplusplus
};
#endif

#endif  /* GUNAVIGATION_ARCS_H */
