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
#include <math.h>

namespace CGTEST {
    
    class TrackingTests: public GUNavigationTests {};


    static void compareStatus(gu_odometry_status expected, gu_odometry_status actual) {
        gu_cartesian_coordinate expectedCoord = expected.my_position.position;
        gu_cartesian_coordinate actualCoord = actual.my_position.position;
        ASSERT_EQ(mm_t_to_i(expectedCoord.x), mm_t_to_i(actualCoord.x));
        ASSERT_EQ(mm_t_to_i(expectedCoord.y), mm_t_to_i(actualCoord.y));
        ASSERT_EQ(mm_t_to_i(expected.last_reading.forward), mm_t_to_i(actual.last_reading.forward));
        ASSERT_EQ(mm_t_to_i(expected.last_reading.left), mm_t_to_i(actual.last_reading.left));
        ASSERT_NEAR(rad_d_to_d(expected.last_reading.turn), rad_d_to_d(actual.last_reading.turn), 0.00001);
        ASSERT_EQ(actual.target.distance, expected.target.distance);
        ASSERT_NEAR(actual.target.direction, expected.target.direction, 0.00001);
    }

    /*static gu_odometry_status getExpected(gu_field_coordinate fieldCoord, gu_cartesian_coordinate coord, gu_odometry_reading lastReading, gu_odometry_reading currentReading)
    {
        const gu_relative_coordinate target = field_coord_to_rr_coord_to_target(fieldCoord, coord);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = calculate_difference(currentReading.forward, currentReading.left, currentReading.turn);
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {544, 1036}; //290 - 104 = 186 |||| 78 + 386 = 464
        const gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {{fieldCoord.position.x + 186, fieldCoord.position.y + 464}, deg_d_to_deg_t(differentialRel.direction) + fieldCoord.heading};
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        compareStatus(expected, actual);       
    }*/

    static void print_status(const gu_odometry_status status)
    {
        printf(
            "Actual((x, y), heading), (r, theta), (forward, left, turn, resetCounter): ((%i, %i), %i), (%i, %0.2f), (%i, %i, %0.2f, %i)\n",
            status.my_position.position.x,
            status.my_position.position.y,
            status.my_position.heading,
            status.target.distance,
            status.target.direction,
            status.last_reading.forward,
            status.last_reading.left,
            status.last_reading.turn,
            status.last_reading.resetCounter
        );
    }

    TEST_F(TrackingTests, CalculateDifference) {
        double forward = 300.0;
        double left = 400.0;
        double turn = rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(45.0)));
        gu_cartesian_coordinate expected = {-71, 495}; //212 - 283, 212 + 283
        gu_cartesian_coordinate actual = calculate_difference(forward, left, turn, 0.0);
        ASSERT_EQ(expected.x, actual.x);
        ASSERT_EQ(expected.y, actual.y);
    }

    TEST_F(TrackingTests, TrackWithReset) {
        const gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        const gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        const gu_cartesian_coordinate coord = {730, 1500};
        const gu_field_coordinate fieldCoord = {{0, 0}, 0};
        const gu_relative_coordinate target = cartesian_coord_to_rr_coord(coord);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = {186, 464};
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {544, 1036}; //290 - 104 = 186 |||| 78 + 386 = 464
        gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {{fieldCoord.position.x + 186, fieldCoord.position.y + 464}, deg_d_to_deg_t(15.0) + fieldCoord.heading};
        expectedTargetRel.direction -= deg_t_to_deg_d(expectedSelf.heading);
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        //print_status(actual);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackWithoutReset) {
        const gu_odometry_reading lastReading = {100, 200, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        const gu_odometry_reading currentReading = {400, 600, deg_d_to_rad_d(d_to_deg_d(35.0)), 0};
        const gu_cartesian_coordinate coord = {730, 1500};
        const gu_field_coordinate fieldCoord = {{0, 0}, 0};
        const gu_relative_coordinate target = cartesian_coord_to_rr_coord(coord);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = {186, 464};
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {544, 1036}; //290 - 104 = 186 |||| 78 + 386 = 464
        gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {{fieldCoord.position.x + 186, fieldCoord.position.y + 464}, deg_d_to_deg_t(15.0) + fieldCoord.heading};
        expectedTargetRel.direction -= deg_t_to_deg_d(expectedSelf.heading);
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        //print_status(actual);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackWithResetWithOriginalTurn) {
        const gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        const gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        const gu_cartesian_coordinate coord = {730, 1500};
        const gu_field_coordinate fieldCoord = {{0, 0}, 30};
        const gu_relative_coordinate target = field_coord_to_rr_coord_to_target(fieldCoord, coord);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = {-71, 495};// 212 - 283, 212 + 283 
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {coord.x - differentialCoord.x, coord.y - differentialCoord.y}; 
        gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {
            {fieldCoord.position.x + differentialCoord.x, fieldCoord.position.y + differentialCoord.y},
            deg_d_to_deg_t(15.0 + 30.0)
        };
        expectedTargetRel.direction -= deg_t_to_deg_d(expectedSelf.heading);
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        //print_status(actual);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackWithResetWithOriginalTurnAndDisplacement) {
        const gu_odometry_reading lastReading = {31231, 432423, deg_d_to_rad_d(d_to_deg_d(20.0)), 0};
        const gu_odometry_reading currentReading = {300, 400, deg_d_to_rad_d(d_to_deg_d(15.0)), 1};
        const gu_cartesian_coordinate coord = {730, 1500};
        const gu_field_coordinate fieldCoord = {{10, 50}, 30};
        const gu_relative_coordinate target = field_coord_to_rr_coord_to_target(fieldCoord, coord);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = {-71, 495};// 212 - 283, 212 + 283 
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {coord.x - differentialCoord.x - fieldCoord.position.x, coord.y - differentialCoord.y - fieldCoord.position.y}; 
        gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {
            {fieldCoord.position.x + differentialCoord.x, fieldCoord.position.y + differentialCoord.y},
            deg_d_to_deg_t(15.0) + fieldCoord.heading
        };
        expectedTargetRel.direction -= deg_t_to_deg_d(expectedSelf.heading);
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        //print_status(actual);
        compareStatus(expected, actual);
    }

    TEST_F(TrackingTests, TrackAnotherWithoutReset)
    {
        const double turn = deg_d_to_rad_d(d_to_deg_d(4.000000476837158));
        const gu_odometry_reading lastReading = {0, 0, deg_d_to_rad_d(d_to_deg_d(0.0)), 0};
        const gu_odometry_reading currentReading = {5, 5, turn, 0};
        const gu_field_coordinate fieldCoord = {{0, 0}, 0};
        const gu_relative_coordinate target = {90.0, 3000};
        const gu_cartesian_coordinate coord = rr_coord_to_cartesian_coord(target);
        const gu_odometry_status currentStatus = {fieldCoord, target, lastReading};
        const gu_cartesian_coordinate differentialCoord = {
            d_to_mm_t(5.0 * cos(turn) + 5.0 * cos(turn + rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(90.0))))),
            d_to_mm_t(5.0 * sin(turn) + 5.0 * sin(turn + rad_d_to_d(deg_d_to_rad_d(d_to_deg_d(90.0)))))
        };
        const gu_relative_coordinate differentialRel = cartesian_coord_to_rr_coord(differentialCoord);
        const gu_cartesian_coordinate expectedTarget = {coord.x - differentialCoord.x - fieldCoord.position.x, coord.y - differentialCoord.y - fieldCoord.position.y}; 
        gu_relative_coordinate expectedTargetRel = cartesian_coord_to_rr_coord(expectedTarget);
        const gu_field_coordinate expectedSelf = {
            {fieldCoord.position.x + differentialCoord.x, fieldCoord.position.y + differentialCoord.y},
            rad_d_to_deg_t(d_to_rad_d(turn))
        };
        expectedTargetRel.direction -= deg_t_to_deg_d(expectedSelf.heading);
        const gu_odometry_status expected = {expectedSelf, expectedTargetRel, currentReading};
        const gu_odometry_status actual = track(currentReading, currentStatus);
        //print_status(actual);
        compareStatus(expected, actual);

    }

    











} //namespace
