/*
 * control.h 
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

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gu_control {
    
    /**
     * The target we are heading towards.
     */
    double target;

    /**
     * The current value.
     */
    double current;

    /**
     * The error between the target and the current value.
     */
    double error;
    
    /**
     * The error before the current iteration of the control algorithm.
     */
    double lastError;

    /**
     * The total error of all iterations of the control algorithm.
     */
    double totalError;

    double controllerOutput;

} gu_control;

typedef struct gu_controller {
    
    /**
     * The proportional gain Kp.
     */
    double proportionalGain;

    /**
     * The derivative gain Kd.
     */
    double derivativeGain;

    /**
     * The integral gain Ki.
     */
    double integralGain;

} gu_controller;

gu_control gu_create_control(const double current, const double target) __attribute__((const));

/**
 * Perform a single iteration of a control algorithm. Algorithms include: proportional, proportional derivative,
 * and proportional integral derivative.
 *
 * You must specify the time between the current and the last iteration dt.
 * The PID algorithm uses error e via: Kp * e + Kd * ((e2 - e1) / dt) + Ki * (allPreviousError + e * dt)
 */
gu_control gu_p_control(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));
gu_control gu_pd_control(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));
gu_control gu_pid_control(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));

gu_control gu_p_control_rel(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));
gu_control gu_pd_control_rel(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));
gu_control gu_pid_control_rel(const gu_control value, const gu_controller controller, const double reading, const double time) __attribute__((const));

/**
 * Operator on gu_control for lhs - rhs.
 *
 * @returns gu_control
 */
gu_control gu_control_relative(const gu_control lhs, const gu_control rhs) __attribute__((const));

gu_control gu_control_add(const gu_control before, const gu_control after) __attribute__((const));

double gu_proportional(const double gain, const double error) __attribute__((const));
double gu_proportional_derivative(const double gain, const double error, const double errorGradient, const double gradientGain) __attribute__((const));
double gu_proportional_integral_derivative(
    const double gain,
    const double error,
    const double errorGradient,
    const double gradientGain,
    const double errorTotal,
    const double integralGain
) __attribute__((const));

#ifdef __cplusplus
}
#endif

#endif  /* CONTROL_H */
