/*
 * control.c 
 * gunavigation 
 *
 * Created by Morgan McColl on 09/10/2020.
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

#include "control.h"

static gu_control makeReading(const gu_control previous, const gu_controller controller, const double reading, const double time, const gu_control_algorithm algorithm)
{
    const double newError = previous.target - reading;
    const double derivativeTerm = (newError - previous.error) / time;
    const double integralTerm = previous.totalError + newError * time;
    double controllerOutput = 0.0;
    switch (algorithm)
    {
        case ControlProportional: controllerOutput = gu_proportional(controller.proportionalGain, newError);
            break;
        case ControlProportionalDerivative: controllerOutput = gu_proportional_derivative(controller.proportionalGain, newError, derivativeTerm, controller.derivativeGain);
            break;
        case ControlProportionalIntegralDerivative:
            controllerOutput = gu_proportional_integral_derivative(controller.proportionalGain, newError, derivativeTerm, controller.derivativeGain, integralTerm, controller.integralGain);
            break;
    }
    gu_control newValue = {
        previous.target,
        reading,
        newError,
        previous.error,
        integralTerm,
        controllerOutput
    };
    return newValue;
}

double gu_proportional(const double gain, const double error)
{
    return gain * error;
}

double gu_proportional_derivative(const double gain, const double error, const double errorGradient, const double gradientGain)
{
    const double p = gu_proportional(gain, error);
    const double d = gradientGain * errorGradient;
    return p + d;
} 

double gu_proportional_integral_derivative(const double gain, const double error, const double errorGradient, const double gradientGain, const double errorTotal, const double integralGain)
{
    const double pd = gu_proportional_derivative(gain, error, errorGradient, gradientGain);
    return pd + integralGain * errorTotal;
} 

gu_control gu_p_control(const gu_control value, const gu_controller controller, const double reading, const double time)
{
    return makeReading(value, controller, reading, time, ControlProportional);
} 

gu_control gu_pd_control(const gu_control value, const gu_controller controller, const double reading, const double time)
{
    return makeReading(value, controller, reading, time, ControlProportionalDerivative);
} 

gu_control gu_pid_control(const gu_control value, const gu_controller controller, const double reading, const double time)
{
    return makeReading(value, controller, reading, time, ControlProportionalIntegralDerivative);
}

gu_control gu_create_control(const double current, const double target)
{
    gu_control control = {
        target,
        current,
        target - current,
        0.0,
        target - current,
        0.0
    };
    return control;
}











