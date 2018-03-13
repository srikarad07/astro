/*
 * Copyright (c) 2018, S.D. Cherukuri, Delft University of Technology (srikarad007@gmail.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <catch.hpp>
#include <Eigen/Eigen>

#include <sml/sml.hpp>

#include "astro/eulerAngleToDcmTransformation.hpp"
#include "astro/eulerKinematicDifferential.hpp"

// #include "astro/loadCsvFile.hpp"

namespace astro
{
namespace test 
{

typedef double Real; 
typedef Eigen::Matrix< double, 3, 1 > Vector3;
typedef Eigen::Matrix< double, 3, 3 > Matrix33; 

TEST_CASE(  "Test Case 1: Euler 3, 2, 1 transformation test for theta_1 = 90 deg, theta_2 = 0 deg, theta_3 = 90 deg", "[rotation angles, rotation_sequence, mean_angular_motion]")
{
    // Benchmark values with know values at the above given values with formula from Analytical Mechanics of Space Systems by Hanspeter Schaub. Without mean motion. 

    const Real tolerance = 1.0e-12;  

    // test for rotation sequence 3-2-1. 
    Matrix33 rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            1, 0, 0;  
    
    // euler angles for the test [rad].  
    Vector3 eulerAngles( sml::convertDegreesToRadians(90.0), sml::convertDegreesToRadians(0.0), sml::convertDegreesToRadians(90.0) );

    // Angular rates: [rad/sec]
    Vector3 angularRates( sml::convertDegreesToRadians(10.0), sml::convertDegreesToRadians(10.0), sml::convertDegreesToRadians(10.0) ); 
    
    // Mean angular motion. 
    const double meanMotion = 0.0; 

    Vector3 computedAttitudeDerivative, expectedAttitudeDerivative; 
    computedAttitudeDerivative = astro::eulerKinematicDifferential( rotationSequence, eulerAngles, angularRates, meanMotion );

    expectedAttitudeDerivative << 0.1745329251994329, -0.1745329251994329, 0.1745329251994329; 

    // Check if computed attitude derivative is the same as the expected attitude derivative. 
    REQUIRE( computedAttitudeDerivative[0] 
                                    == Approx(expectedAttitudeDerivative[0]).epsilon(tolerance) ); 
    REQUIRE( computedAttitudeDerivative[1] 
                                    == Approx(expectedAttitudeDerivative[1]).epsilon(tolerance) ); 
    REQUIRE( computedAttitudeDerivative[2] 
                                    == Approx(expectedAttitudeDerivative[2]).epsilon(tolerance) ); 
}


TEST_CASE(  "Test Case 2: Euler 3, 2, 1 transformation test for theta_1 = 90 deg, theta_2 = 0 deg, theta_3 = 90 deg", "[rotation angles, rotation_sequence, mean_angular_motion]")
{
    // Benchmark values with know values at the above given values with formula from Analytical Mechanics of Space Systems by Hanspeter Schaub. With mean motion. 

    const Real tolerance = 1.0e-12;  

    // test for rotation sequence 3-2-1. 
    Matrix33 rotationSequence; 
    rotationSequence <<     0, 0, 1, 
                            0, 1, 0, 
                            1, 0, 0;  
    
    // euler angles for the test [rad].  
    Vector3 eulerAngles( sml::convertDegreesToRadians(90.0), sml::convertDegreesToRadians(0.0), sml::convertDegreesToRadians(90.0) );

    // Angular rates: [rad/sec]
    Vector3 angularRates( sml::convertDegreesToRadians(10.0), sml::convertDegreesToRadians(10.0), sml::convertDegreesToRadians(10.0) ); 
    
    // Mean angular motion. 
    const double meanMotion = 10.0; 

    Vector3 computedAttitudeDerivative, expectedAttitudeDerivative; 
    computedAttitudeDerivative = astro::eulerKinematicDifferential( rotationSequence, eulerAngles, angularRates, meanMotion );

    expectedAttitudeDerivative << 1.017453292519943e+01, -0.1745329251994329, 0.1745329251994329; 

    // Check if computed attitude derivative is the same as the expected attitude derivative. 
    REQUIRE( computedAttitudeDerivative[0] 
                                    == Approx(expectedAttitudeDerivative[0]).epsilon(tolerance) ); 
    REQUIRE( computedAttitudeDerivative[1] 
                                    == Approx(expectedAttitudeDerivative[1]).epsilon(tolerance) ); 
    REQUIRE( computedAttitudeDerivative[2] 
                                    == Approx(expectedAttitudeDerivative[2]).epsilon(tolerance) ); 
}

} // namespace astro

} // namespace test