/*
 * tracking_tests.cc
 * Copyright (C) 2020 Morgan McColl <morgan.mccoll@alumni.griffithuni.edu.au>
 *
 * Distributed under terms of the MIT license.
 */

#include "gunavigation_tests.hpp"
#include <guunits/guunits.h>
#include <gucoordinates/gucoordinates.h>
#include <stdio.h>

namespace CGTEST {
    
    class TrackingTests: public GUNavigationTests {};


    static void compareStatus(gu_odometry_status expected, gu_odometry_status actual) {
        gu_cartesian_coordinate expectedCoord = to_cartesian_coordinate(expected);
        gu_cartesian_coordinate actualCoord = to_cartesian_coordinate(actual);
        ASSERT_EQ(mm_t_to_i(expected.forward), mm_t_to_i(actual.forward));
        ASSERT_EQ(mm_t_to_i(expected.left), mm_t_to_i(actual.left));
        ASSERT_NEAR(rad_d_to_d(expected.turn), rad_d_to_d(actual.turn), 0.00001);
        ASSERT_EQ(cm_t_to_i(expectedCoord.x), cm_t_to_i(actualCoord.x));
        ASSERT_EQ(cm_t_to_i(expectedCoord.y), cm_t_to_i(actualCoord.y));
    }

    TEST_F(TrackingTests, CalculateDifference) {
        double forward = 300.0;
        double left = 400.0;
        double turn = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(45.0)));
        gu_cartesian_coordinate expected = {-7, 50};
        gu_cartesian_coordinate actual = calculate_difference(forward, left, turn);
        ASSERT_EQ(expected.x, actual.x);
        ASSERT_EQ(expected.y, actual.y);
    }

    TEST_F(TrackingTests, TrackWithReset) {
        gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        gu_cartesian_coordinate coord = {73, 150};
        gu_odometry_status currentStatus = {100, 20, deg_d_to_rad_d(d_to_deg_d(10.0)), &coord};
        gu_cartesian_coordinate expectedCoord = {80, 100};
        gu_odometry_status expected = {400, 420, deg_d_to_rad_d(d_to_deg_d(30.0)), &expectedCoord};
        gu_odometry_status actual = track_coordinate(lastReading, currentReading, currentStatus);
        gu_cartesian_coordinate actualCoord = to_cartesian_coordinate(actual);
        compareStatus(expected, actual);
    }
} //namespace
