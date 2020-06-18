/*
 * conversions.c 
 * gunavigation 
 *
 * Created by Callum McColl on 19/06/2020.
 * Copyright © 2020 Callum McColl. All rights reserved.
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

#include "conversions.h"

#include <guvision_utils/guvision_utils.h>
#include <math.h>

bool px_coord_to_rr_coord(const gu_pixel_coordinate coord, const gu_robot robot, relative_coordinate * out)
{
    return pct_coord_to_rr_coord(px_coord_to_pct_coord(coord), robot, out);
}

bool px_coord_to_rr_coord_cam(const gu_pixel_coordinate coord, const gu_robot robot, relative_coordinate * out, const int cameraOffset)
{
    return pct_coord_to_rr_coord_cam(px_coord_to_pct_coord(coord), robot, out, cameraOffset);
}

bool pct_coord_to_rr_coord(const gu_percent_coordinate coord, const gu_robot robot, relative_coordinate * out)
{
    for (int i = 0; i < robot.numCameras; i++) {
        if (pct_coord_to_rr_coord_cam(coord, robot, out, i))
        {
            return true;
        }
    }
    return false;
}

bool pct_coord_to_rr_coord_cam(const gu_percent_coordinate coord, const gu_robot robot, relative_coordinate * out, const int cameraOffset)
{
    const gu_camera camera = robot.cameras[cameraOffset];
    const degrees_f pitch = robot.headPitch + camera.vDirection + f_to_deg_f(pct_f_to_f(coord.y)) * (camera.vFov / 2.0f);
    if (pitch >= 90.0f)
    {
        return false;
    }
    const degrees_f yaw = robot.headYaw + f_to_deg_f(pct_f_to_f(coord.x)) * (camera.hFov / 2.0f);
    const radians_f pitchRad = deg_f_to_rad_f(pitch);
    const radians_f yawRad = deg_f_to_rad_f(yaw);
    const float distance = cm_f_to_f(camera.height) * tanf(rad_f_to_f(pitchRad)) / cosf(rad_f_to_f(yawRad));
    out->distance = f_to_cm_u(fabsf(distance));
    out->direction = deg_f_to_deg_t(yaw);
    return true;
}
