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
        gu_cartesian_coordinate expectedCoord = expected.cartesian_coordinate;
        gu_cartesian_coordinate actualCoord = actual.cartesian_coordinate;
        ASSERT_EQ(mm_t_to_i(expected.forward), mm_t_to_i(actual.forward));
        ASSERT_EQ(mm_t_to_i(expected.left), mm_t_to_i(actual.left));
        ASSERT_NEAR(rad_d_to_d(expected.turn), rad_d_to_d(actual.turn), 0.00001);
        ASSERT_EQ(cm_t_to_i(expectedCoord.x), cm_t_to_i(actualCoord.x));
        ASSERT_EQ(cm_t_to_i(expectedCoord.y), cm_t_to_i(actualCoord.y));
        ASSERT_EQ(mm_t_to_i(expected.last_reading.forward), mm_t_to_i(actual.last_reading.forward));
        ASSERT_EQ(mm_t_to_i(expected.last_reading.left), mm_t_to_i(actual.last_reading.left));
        ASSERT_NEAR(rad_d_to_d(expected.last_reading.turn), rad_d_to_d(actual.last_reading.turn), 0.00001);
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

    TEST_F(TrackingTests, CalculateDifferenceRelative) {
        double forward = 300.0;
        double left = 400.0;
        double turn = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(45.0)));
        gu_cartesian_coordinate expectedCartesian = {-7, 50};
        gu_relative_coordinate expected = cartesian_coord_to_rr_coord(expectedCartesian);
        gu_relative_coordinate actual = calculate_difference_relative(forward, left, turn);
        ASSERT_EQ(expected.distance, actual.distance);
        ASSERT_EQ(expected.direction, actual.direction);

    }

    TEST_F(TrackingTests, TrackWithReset) {
        gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        gu_cartesian_coordinate coord = {73, 150};
        gu_odometry_status currentStatus = {100, 20, deg_d_to_rad_d(d_to_deg_d(10.0)), coord, {}, lastReading};
        gu_cartesian_coordinate expectedCoord = {80, 100};
        gu_odometry_status expected = {300, 400, deg_d_to_rad_d(d_to_deg_d(30.0)), expectedCoord, {}, currentReading};
        gu_odometry_status actual = track_coordinate(currentReading, currentStatus);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackWithoutReset) {
        gu_odometry_reading lastReading = {100, 200, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {400, 600, deg_d_to_rad_d(d_to_deg_d(35.0)), 0};
        gu_cartesian_coordinate coord = {73, 150};
        gu_odometry_status currentStatus = {100, 200, deg_d_to_rad_d(d_to_deg_d(10.0)), coord, {}, lastReading};
        gu_cartesian_coordinate expectedCoord = {80, 100}; 
        gu_odometry_status expected = {400, 600, deg_d_to_rad_d(d_to_deg_d(10.0)), expectedCoord, {}, currentReading};
        gu_odometry_status actual = track_coordinate(currentReading, currentStatus);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackRelativeCoordinate) {
        gu_odometry_reading lastReading = {100, 200, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {400, 600, deg_d_to_rad_d(d_to_deg_d(35.0)), 0};
        gu_cartesian_coordinate coord = {73, 150};
        gu_relative_coordinate relCoord = cartesian_coord_to_rr_coord(coord); 
        gu_odometry_status currentStatus = {100, 200, deg_d_to_rad_d(d_to_deg_d(10.0)), {}, relCoord, lastReading};
        gu_cartesian_coordinate expectedCoord = {80, 100}; 
        gu_relative_coordinate expectedRelative = cartesian_coord_to_rr_coord(expectedCoord);
        gu_odometry_status expected = {400, 600, deg_d_to_rad_d(d_to_deg_d(10.0)), expectedCoord, expectedRelative, currentReading};
        gu_odometry_status actual = track_relative_coordinate(currentReading, currentStatus);
        compareStatus(expected, actual);
        gu_relative_coordinate actualRelative = actual.relative_coordinate;
        ASSERT_EQ(mm_u_to_i(expectedRelative.distance), mm_u_to_i(actualRelative.distance));
        ASSERT_NEAR(deg_d_to_d(expectedRelative.direction), deg_d_to_d(actualRelative.direction), 0.00001);
    }

    TEST_F(TrackingTests, TrackSelf) {
        gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        gu_cartesian_coordinate coord = {73, 150};
        gu_odometry_status currentStatus = {100, 20, deg_d_to_rad_d(d_to_deg_d(10.0)), coord, {}, lastReading};
        gu_cartesian_coordinate expectedCoord = {66, 200};
        gu_odometry_status expected = {300, 400, deg_d_to_rad_d(d_to_deg_d(30.0)), expectedCoord, {}, currentReading};
        gu_odometry_status actual = track_self(currentReading, currentStatus);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackSelfRelative) {
        gu_odometry_reading lastReading = {100, 200, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        gu_odometry_reading currentReading = {400, 600, deg_d_to_rad_d(d_to_deg_d(35.0)), 0};
        gu_cartesian_coordinate coord = {73, 150};
        gu_relative_coordinate relCoord = cartesian_coord_to_rr_coord(coord); 
        gu_odometry_status currentStatus = {100, 200, deg_d_to_rad_d(d_to_deg_d(10.0)), {}, relCoord, lastReading};
        gu_cartesian_coordinate expectedCoord = {66, 200}; 
        gu_relative_coordinate expectedRelative = cartesian_coord_to_rr_coord(expectedCoord);
        gu_odometry_status expected = {400, 600, deg_d_to_rad_d(d_to_deg_d(10.0)), expectedCoord, expectedRelative, currentReading};
        gu_odometry_status actual = track_self_relative(currentReading, currentStatus);
        compareStatus(expected, actual);
        gu_relative_coordinate actualRelative = actual.relative_coordinate;
        ASSERT_EQ(mm_u_to_i(expectedRelative.distance), mm_u_to_i(actualRelative.distance));
        ASSERT_NEAR(deg_d_to_d(expectedRelative.direction), deg_d_to_d(actualRelative.direction), 0.00001);

    }














} //namespace
