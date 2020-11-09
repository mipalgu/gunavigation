/*
 * GUCoordinatesTests.hpp 
 * tests 
 *
 * Created by Callum McColl on 20/06/2020.
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

#ifndef GUNAVIGATIONTESTS_HPP
#define GUNAVIGATIONTESTS_HPP

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma clang diagnostic ignored "-Wfloat-equal"
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#pragma clang diagnostic ignored "-Wshift-sign-overflow"
#pragma clang diagnostic ignored "-Wused-but-marked-unused"
#pragma clang diagnostic ignored "-Wundef"
#pragma clang diagnostic ignored "-Wc++98-compat"
#pragma clang diagnostic ignored "-Wc++98-compat-pedantic"
#pragma clang diagnostic ignored "-Wc++11-long-long"
#pragma clang diagnostic ignored "-Wc++11-extensions"
#pragma clang diagnostic ignored "-Wdeprecated"
#include <gtest/gtest.h>
#pragma clang diagnostic pop

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++98-compat-pedantic"
#include "fff.h"
#pragma clang diagnostic pop

#include "../gunavigation.h"
#include "fakes.h"
#include "custom_fakes.h"

#include <typeinfo>

#include <math.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"

#define TEST2_F(testclassname, testname) \
    TEST_F(testclassname, testname)

#define RO3_TEST_F(className) \
    TEST2_F(testclassname(className), RO3) { \
        ro3_test(); \
    }

#if __cplusplus >= 201103L
#define RO5_TEST_Fs(className) \
    RO3_TEST_F(className) \
    TEST2_F(testclassname(className), RO5) { \
        ro5_test(); \
    }
#else
#define RO5_TEST_Fs(className) RO3_TEST_F(className)
#endif

#define testclassname(className) className##CPPTests
#define equals_reset(strctName) strctName##_equals_reset();
#define equals_func(strctName) strctName##_equals
#define equals_fake(strctName) strctName##_equals_fake

#pragma clang diagnostic pop

namespace CGTEST {

    template <typename Class>
    class GUNavigationTests: public ::testing::Test {
        private:

        protected:

            virtual void SetUp() {
                ALL_FAKES(RESET_FAKE);
                FFF_RESET_HISTORY();
            }

            virtual void TearDown() {
                ALL_FAKES(RESET_FAKE);
                FFF_RESET_HISTORY();
            }

            virtual void preamble() {}

            virtual Class initial() = 0;

            virtual void change(Class &) {}

            virtual Class empty() = 0;

            void ro3_test() {
                preamble();
                Class obj = initial();
#pragma clang diangostic push
#pragma clang diagnostic ignored "-Wself-assign"
                obj = obj;
#pragma clang diagnostic pop
                SCOPED_TRACE("Equality operator");
                equals(obj, obj);
                SCOPED_TRACE("Copy constructor");
                Class obj2 = Class(obj);
                equals(obj, obj2);
                SCOPED_TRACE("Assignment operator");
                Class obj3 = obj2;
                equals(obj, obj3);
                change(obj);
                nequals(obj, obj3);
                equals(obj2, obj3);
                SCOPED_TRACE("Assignment operator2");
                Class obj4;
                obj4 = obj3;
                equals(obj4, obj3);
                Class * obj5 = &obj4;
                equals(obj4, *obj5);
            }

#if __cplusplus >= 201103L
            void ro5_test() {
                Class obj = initial();
                Class obj2 = initial();
                SCOPED_TRACE("Move Constructor");
                Class obj3 = std::move(obj);
                nequals(obj3, obj);
                equals(obj3, obj2);
                equals(obj, empty());
                SCOPED_TRACE("Move assignment operator");
                Class obj4;
                obj4 = std::move(obj3);
                nequals(obj4, obj3);
                equals(obj4, obj2);
                equals(obj3, empty());
                SCOPED_TRACE("Move assignment operator2");
                Class * obj5 = &obj4;
                obj4 = std::move(*obj5);
                nequals(obj4, obj3);
                equals(obj4, obj2);
                nequals(*obj5, obj3);
                equals(*obj5, obj2);
                equals(*obj5, obj4);
            }
#endif

            bool near(const double lhs, const double rhs) const
            {
                const double tolerance = 0.00001;
                return fabs(lhs - rhs) < tolerance;
            }

            void assert_near(const double lhs, const double rhs, const double tolerance) const
            {
                ASSERT_LE(fabs(lhs - rhs), tolerance);
            }

    };

}

#endif  /* GUCOORDINATESTESTS_HPP */
