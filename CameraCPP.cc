/*
 * Camera.cc 
 * guvision_utils 
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

#include "Camera.hpp"

GU::Camera::Camera(): gu_camera() {}

GU::Camera::Camera(centimetres_f t_height, centimetres_f t_centerOffset, degrees_f t_vDirection, degrees_f t_vFov, degrees_f t_hFov): gu_camera { t_height, t_centerOffset, t_vDirection, t_vFov, t_hFov } {} 

GU::Camera::Camera(const Camera& other): gu_camera { other.height(), other.centerOffset(), other.vDirection(), other.vFov(), other.hFov() } {}

#if __cplusplus >= 199711L
GU::Camera::Camera(Camera&& other)
{
    set_height(other.height());
    set_centerOffset(other.centerOffset());
    set_vDirection(other.vDirection());
    set_vFov(other.vFov());
    set_hFov(other.hFov());
    other.set_height(0.0f);
    other.set_centerOffset(0.0f);
    other.set_vDirection(0.0f);
    other.set_vFov(0.0f);
    other.set_hFov(0.0f);
}
#endif

GU::Camera::~Camera() {}

GU::Camera& GU::Camera::operator=(const Camera& other)
{
    if (&other == this)
    {
        return *this;
    }
    set_height(other.height());
    set_centerOffset(other.centerOffset());
    set_vDirection(other.vDirection());
    set_vFov(other.vFov());
    set_hFov(other.hFov());
    return *this;
}

#if __cplusplus >= 199711L
GU::Camera& GU::Camera::operator=(Camera&& other)
{
    if (&other == this) {
        return *this;
    }
    set_height(other.height());
    set_centerOffset(other.centerOffset());
    set_vDirection(other.vDirection());
    set_vFov(other.vFov());
    set_hFov(other.hFov());
    other.set_height(0.0f);
    other.set_centerOffset(0.0f);
    other.set_vDirection(0.0f);
    other.set_vFov(0.0f);
    other.set_hFov(0.0f);
    return *this;
}
#endif

centimetres_f GU::Camera::height() const
{
    return gu_camera::height;
}

void GU::Camera::set_height(const centimetres_f newValue)
{
    gu_camera::height = newValue;
}

centimetres_f GU::Camera::centerOffset() const
{
    return gu_camera::centerOffset;
}

void GU::Camera::set_centerOffset(const centimetres_f newValue)
{
    gu_camera::centerOffset = newValue;
}

degrees_f GU::Camera::vDirection() const
{
    return gu_camera::vDirection;
}

void GU::Camera::set_vDirection(const degrees_f newValue)
{
    gu_camera::vDirection = newValue;
}

degrees_f GU::Camera::vFov() const
{
    return gu_camera::vFov;
}

void GU::Camera::set_vFov(const degrees_f newValue)
{
    gu_camera::vFov = newValue;
}

degrees_f GU::Camera::hFov() const
{
    return gu_camera::hFov;
}

void GU::Camera::set_hFov(const degrees_f newValue)
{
    gu_camera::hFov = newValue;
}

bool GU::Camera::operator==(const Camera &other) const
{
    return gu_camera_equals(*this, other, 0.0001f);
}

bool GU::Camera::operator!=(const Camera &other) const
{
    return !(*this == other);
}
