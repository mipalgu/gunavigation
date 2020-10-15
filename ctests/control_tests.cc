/*
 * camera_tests.cc 
 * tests 
 *
 * Created by Callum McColl on 18/06/2020.
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

#include "gunavigation_tests.hpp"

namespace CGTEST {
    
    class ControlTests: public GUNavigationTests {};

    TEST_F(ControlTests, Proportional) {
        const double expected = 1.0;
        const double actual = gu_proportional(0.5, 2.0);
        ASSERT_NEAR(actual, expected, 0.00001);
    }

    TEST_F(ControlTests, ProportionalDerivative) {
        const double target = 6.0;
        const double original = 1.0;
        double current = original; //1.0
        double error = target - current; //5.0
        double errorGradient = error; //5.0
        const double proportionalGain = 0.5;
        const double gradientGain = 0.1;
        const double expected = 3.0;
        const double actual = gu_proportional_derivative(proportionalGain, error, errorGradient, gradientGain);
        ASSERT_NEAR(actual, expected, 0.00001);
        current += actual; //4.0
        ASSERT_NEAR(current, 4.0, 0.00001);
        double lastError = error; //5.0
        error = target - current; //2.0
        errorGradient = error - lastError; // -3.0
        const double expected2 = 0.7;
        const double actual2 = gu_proportional_derivative(proportionalGain, error, errorGradient, gradientGain);
        ASSERT_NEAR(actual2, expected2, 0.00001);
        current += actual2; //4.7
        lastError = error; // 2.0
        error = target - current; // 1.3
        errorGradient = error - lastError; // -0.7
        const double expected3 = 0.58;
        const double actual3 = gu_proportional_derivative(proportionalGain, error, errorGradient, gradientGain);
        ASSERT_NEAR(actual3, expected3, 0.00001);
    }

    TEST_F(ControlTests, ProportionalIntegralDerivative) {
        const double target = 6.0;
        const double original = 1.0;
        double current = original;
        double error = target - current; //5.0
        double errorGradient = error; //5.0
        double totalError = error; //5.0
        const double proportionalGain = 0.5;
        const double gradientGain = 0.1;
        const double integralGain = 0.1;
        const double expected = 3.5;
        const double actual = gu_proportional_integral_derivative(proportionalGain, error, errorGradient, gradientGain, totalError, integralGain);
        ASSERT_NEAR(expected, actual, 0.00001);
        current += actual; //4.5
        double lastError = error; //5.0
        error = target - current; // 1.5
        errorGradient = error - lastError; // -3.5
        totalError += error; // 6.5
        const double expected2 = 1.05;
        const double actual2 = gu_proportional_integral_derivative(proportionalGain, error, errorGradient, gradientGain, totalError, integralGain);
        ASSERT_NEAR(expected2, actual2, 0.00001);
        current += actual2; // 5.55
        lastError = error; // 1.5
        error = target - current; // 0.45
        errorGradient = error - lastError; // -1.05
        totalError += error; // 6.95
        const double expected3 = 0.815;
        const double actual3 = gu_proportional_integral_derivative(proportionalGain, error, errorGradient, gradientGain, totalError, integralGain);
        ASSERT_NEAR(expected3, actual3, 0.00001);
        current += actual3; // 6.365
        lastError = error; // 0.45
        error = target - current; // -0.365
        errorGradient = error - lastError; // -0.815
        totalError += error; // 6.585
        const double expected4 = 0.3945;
        const double actual4 = gu_proportional_integral_derivative(proportionalGain, error, errorGradient, gradientGain, totalError, integralGain);
        ASSERT_NEAR(expected4, actual4, 0.00001);
    }

    TEST_F(ControlTests, PControl) {
        const gu_control val = gu_create_control(0.0, 6.0); 
        const gu_controller controller = {0.5, 0.0, 0.0};
        const double dt = 1.0;
        const gu_control expected = {6.0, 5.0, 1.0, 6.0, 1.0, 0.5};
        const gu_control actual = gu_p_control(val, controller, 5.0, dt);
        ASSERT_NEAR(expected.target, actual.target, 0.00001);
        ASSERT_NEAR(expected.current, actual.current, 0.00001);
        ASSERT_NEAR(expected.error, actual.error, 0.00001);
        ASSERT_NEAR(expected.lastError, actual.lastError, 0.00001);
        ASSERT_NEAR(expected.totalError, actual.totalError, 0.00001);
        ASSERT_NEAR(expected.controllerOutput, actual.controllerOutput, 0.00001);
    }

    TEST_F(ControlTests, PDControl) {
        const gu_control val = gu_create_control(0.0, 6.0); 
        const gu_controller controller = {0.5, 0.1, 0.0};
        const double dt = 0.5;
        const gu_control expected = {6.0, 5.0, 1.0, 6.0, 0.5, -0.5};
        const gu_control actual = gu_pd_control(val, controller, 5.0, dt);
        ASSERT_NEAR(expected.target, actual.target, 0.00001);
        ASSERT_NEAR(expected.current, actual.current, 0.00001);
        ASSERT_NEAR(expected.error, actual.error, 0.00001);
        ASSERT_NEAR(expected.lastError, actual.lastError, 0.00001);
        ASSERT_NEAR(expected.totalError, actual.totalError, 0.00001);
        ASSERT_NEAR(expected.controllerOutput, actual.controllerOutput, 0.00001);
    }

    TEST_F(ControlTests, PIDControl) {
        const gu_control val = gu_create_control(0.0, 6.0); 
        const gu_controller controller = {0.5, 0.1, 0.1};
        const double dt = 0.5;
        const gu_control expected = {6.0, 5.0, 1.0, 6.0, 0.5, -0.45};
        const gu_control actual = gu_pid_control(val, controller, 5.0, dt);
        ASSERT_NEAR(expected.target, actual.target, 0.00001);
        ASSERT_NEAR(expected.current, actual.current, 0.00001);
        ASSERT_NEAR(expected.error, actual.error, 0.00001);
        ASSERT_NEAR(expected.lastError, actual.lastError, 0.00001);
        ASSERT_NEAR(expected.totalError, actual.totalError, 0.00001);
        ASSERT_NEAR(expected.controllerOutput, actual.controllerOutput, 0.00001);
    }

}  // namespace
